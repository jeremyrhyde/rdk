package datamanager_test

import (
	"context"
	"io/fs"
	"io/ioutil"
	"os"
	"sync"
	"testing"
	"time"

	"github.com/edaniels/golog"
	"github.com/pkg/errors"
	v1 "go.viam.com/api/proto/viam/datasync/v1"
	"go.viam.com/test"

	"go.viam.com/rdk/component/arm"
	"go.viam.com/rdk/config"
	commonpb "go.viam.com/rdk/proto/api/common/v1"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot"
	"go.viam.com/rdk/services/datamanager"
	"go.viam.com/rdk/services/datamanager/internal"
	"go.viam.com/rdk/testutils/inject"
	rutils "go.viam.com/rdk/utils"
)

// readDir filters out folders from a slice of FileInfos.
func readDir(t *testing.T, dir string) ([]fs.FileInfo, error) {
	t.Helper()
	filesAndFolders, err := ioutil.ReadDir(dir)
	if err != nil {
		t.Log(err)
	}
	var onlyFiles []fs.FileInfo
	for _, s := range filesAndFolders {
		if !s.IsDir() {
			onlyFiles = append(onlyFiles, s)
		}
	}
	return onlyFiles, err
}

func resetFolder(t *testing.T, path string) {
	t.Helper()
	if err := os.RemoveAll(path); err != nil {
		t.Log(err)
	}
}

func getInjectedRobotWithArm(armKey string) *inject.Robot {
	r := &inject.Robot{}
	rs := map[resource.Name]interface{}{}
	injectedArm := &inject.Arm{}
	injectedArm.GetEndPositionFunc = func(ctx context.Context) (*commonpb.Pose, error) {
		return &commonpb.Pose{X: 1, Y: 2, Z: 3}, nil
	}
	rs[arm.Named(armKey)] = injectedArm
	r.MockResourcesFromMap(rs)
	return r
}

func newTestDataManager(t *testing.T, localArmKey string, remoteArmKey string) internal.DMService {
	t.Helper()
	dmCfg := &datamanager.Config{}
	cfgService := config.Service{
		Type:                "data_manager",
		ConvertedAttributes: dmCfg,
	}
	logger := golog.NewTestLogger(t)

	// Create local robot with injected arm.
	r := getInjectedRobotWithArm(localArmKey)

	// If passed, create remote robot with an injected arm.
	if remoteArmKey != "" {
		remoteRobot := getInjectedRobotWithArm(remoteArmKey)

		r.RemoteByNameFunc = func(name string) (robot.Robot, bool) {
			return remoteRobot, true
		}
	}

	svc, err := datamanager.New(context.Background(), r, cfgService, logger)
	if err != nil {
		t.Log(err)
	}
	return svc.(internal.DMService)
}

func setupConfig(t *testing.T, relativePath string) *config.Config {
	t.Helper()
	logger := golog.NewTestLogger(t)
	testCfg, err := config.Read(context.Background(), rutils.ResolveFile(relativePath), logger)
	test.That(t, err, test.ShouldBeNil)
	testCfg.Cloud = &config.Cloud{ID: "part_id"}
	return testCfg
}

func TestNewDataManager(t *testing.T) {
	// Empty config at initialization.
	captureDir := "/tmp/capture"
	svc := newTestDataManager(t, "arm1", "")
	// Set capture parameters in Update.
	conf := setupConfig(t, "robots/configs/fake_robot_with_data_manager.json")
	svcConfig, ok, err := datamanager.GetServiceConfig(conf)
	test.That(t, ok, test.ShouldBeTrue)
	test.That(t, err, test.ShouldBeNil)
	defer resetFolder(t, captureDir)
	svc.Update(context.Background(), conf)
	sleepTime := time.Millisecond * 5
	time.Sleep(sleepTime)

	// Verify that the single configured collector wrote to its file.
	files, err := ioutil.ReadDir(svcConfig.CaptureDir)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(files), test.ShouldEqual, 1)

	// Verify that after close is called, the collector is no longer writing.
	oldSize := files[0].Size()
	err = svc.Close(context.Background())
	// When Close returns all background processes in svc should be closed, but still sleep for 100ms to verify
	// that there's not a resource leak causing writes to still happens after Close() returns.
	time.Sleep(time.Millisecond * 100)
	test.That(t, err, test.ShouldBeNil)
	newSize := files[0].Size()
	test.That(t, oldSize, test.ShouldEqual, newSize)
}

func TestNewRemoteDataManager(t *testing.T) {
	// Empty config at initialization.
	captureDir := "/tmp/capture"
	svc := newTestDataManager(t, "localArm", "remoteArm")

	// Set capture parameters in Update.
	conf := setupConfig(t, "robots/configs/fake_robot_with_remote_and_data_manager.json")
	defer resetFolder(t, captureDir)
	svc.Update(context.Background(), conf)
	sleepTime := time.Millisecond * 100
	time.Sleep(sleepTime)

	// Verify that after close is called, the collector is no longer writing.
	err := svc.Close(context.Background())
	test.That(t, err, test.ShouldBeNil)

	// Verify that the local and remote collectors wrote to their files.
	localArmDir := captureDir + "/arm/localArm/GetEndPosition"
	filesInLocalArmDir, err := readDir(t, localArmDir)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(filesInLocalArmDir), test.ShouldEqual, 1)
	test.That(t, filesInLocalArmDir[0].Size(), test.ShouldBeGreaterThan, 0)

	remoteArmDir := captureDir + "/arm/remoteArm/GetEndPosition"
	filesInRemoteArmDir, err := readDir(t, remoteArmDir)
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(filesInRemoteArmDir), test.ShouldEqual, 1)
	test.That(t, filesInRemoteArmDir[0].Size(), test.ShouldBeGreaterThan, 0)
}

// Validates that manual syncing works for a datamanager.
func TestManualSync(t *testing.T) {
	var uploaded []string
	lock := sync.Mutex{}
	uploadFn := func(ctx context.Context, client v1.DataSyncService_UploadClient, path string, partID string) error {
		lock.Lock()
		uploaded = append(uploaded, path)
		lock.Unlock()
		return nil
	}
	configPath := "robots/configs/fake_data_manager.json"
	testCfg := setupConfig(t, configPath)

	// Make the captureDir where we're logging data for our arm.
	captureDir := "/tmp/capture"
	armDir := captureDir + "/arm/arm1/GetEndPosition"

	// Clear the capture dir after we're done.
	defer resetFolder(t, armDir)

	// Initialize the data manager and update it with our config.
	dmsvc := newTestDataManager(t, "arm1", "")
	defer dmsvc.Close(context.Background())
	dmsvc.SetUploadFn(uploadFn)
	dmsvc.Update(context.Background(), testCfg)

	// Run and upload files.
	dmsvc.Sync(context.Background())
	time.Sleep(time.Millisecond * 100)

	// Verify that the file was uploaded.
	lock.Lock()
	test.That(t, len(uploaded), test.ShouldEqual, 1)
	lock.Unlock()

	// Do it again and verify it synced the second file, but not the first again.
	dmsvc.Sync(context.Background())
	time.Sleep(time.Millisecond * 100)
	_ = dmsvc.Close(context.TODO())
	test.That(t, len(uploaded), test.ShouldEqual, 2)
	test.That(t, uploaded[0], test.ShouldNotEqual, uploaded[1])
}

// Validates that scheduled syncing works for a datamanager.
func TestScheduledSync(t *testing.T) {
	uploaded := []string{}
	lock := sync.Mutex{}
	uploadFn := func(ctx context.Context, client v1.DataSyncService_UploadClient, path string, partID string) error {
		lock.Lock()
		uploaded = append(uploaded, path)
		lock.Unlock()
		return nil
	}
	configPath := "robots/configs/fake_data_manager.json"
	testCfg := setupConfig(t, configPath)
	svcConfig, ok, err := datamanager.GetServiceConfig(testCfg)
	if !ok {
		t.Error("malformed/missing datamanager service in config")
	}
	if err != nil {
		t.Error(err)
	}
	svcConfig.SyncIntervalMins = 0.0041

	// Make the captureDir where we're logging data for our arm.
	captureDir := "/tmp/capture"
	armDir := captureDir + "/arm/arm1/GetEndPosition"

	// Clear the capture dir after we're done.
	defer resetFolder(t, armDir)

	// Initialize the data manager and update it with our config.
	dmsvc := newTestDataManager(t, "arm1", "")
	dmsvc.SetUploadFn(uploadFn)
	dmsvc.Update(context.TODO(), testCfg)

	// We set sync_interval_mins to be about 250ms in the config, so wait 600ms and ensure two files were uploaded.
	time.Sleep(time.Millisecond * 600)
	dmsvc.Close(context.TODO())
	test.That(t, len(uploaded), test.ShouldEqual, 2)
	test.That(t, uploaded[0], test.ShouldNotEqual, uploaded[1])
}

// Validates that we can attempt a scheduled and manual syncDataCaptureFiles at the same time without duplicating files
// or running into errors.
func TestManualAndScheduledSync(t *testing.T) {
	var uploadedFiles []string
	lock := sync.Mutex{}
	uploadFn := func(ctx context.Context, client v1.DataSyncService_UploadClient, path string, partID string) error {
		lock.Lock()
		uploadedFiles = append(uploadedFiles, path)
		lock.Unlock()
		return nil
	}
	// Use config with 250ms sync interval.
	configPath := "robots/configs/fake_data_manager.json"
	testCfg := setupConfig(t, configPath)
	err := setConfigIntervalMins(testCfg, 0.0041)
	test.That(t, err, test.ShouldBeNil)

	// Make the captureDir where we're logging data for our arm.
	captureDir := "/tmp/capture"
	armDir := captureDir + "/arm/arm1/GetEndPosition"

	// Clear the capture dir after we're done.
	defer resetFolder(t, armDir)

	// Initialize the data manager and update it with our config.
	dmsvc := newTestDataManager(t, "arm1", "")

	// Make sure we close resources to prevent leaks.
	dmsvc.SetUploadFn(uploadFn)
	dmsvc.Update(context.TODO(), testCfg)

	// Perform a manual and scheduled syncDataCaptureFiles at approximately the same time, then close the svc.
	time.Sleep(time.Millisecond * 250)
	dmsvc.Sync(context.TODO())
	time.Sleep(time.Millisecond * 100)
	_ = dmsvc.Close(context.TODO())

	// Verify two files were uploaded, and that they're different.
	test.That(t, len(uploadedFiles), test.ShouldEqual, 2)
	test.That(t, uploadedFiles[0], test.ShouldNotEqual, uploadedFiles[1])

	// We've uploaded (and thus deleted) the first two files and should now be collecting a single new one.
	filesInArmDir, err := readDir(t, armDir)
	if err != nil {
		t.Fatalf("failed to list files in armDir")
	}
	test.That(t, len(filesInArmDir), test.ShouldEqual, 1)
}

// Validates that if the datamanager/robot die unexpectedly, that previously captured but not synced files are still
// synced at start up.
func TestRecoversAfterKilled(t *testing.T) {
	uploaded := []string{}
	lock := sync.Mutex{}
	uploadFn := func(ctx context.Context, client v1.DataSyncService_UploadClient, path string, partID string) error {
		lock.Lock()
		uploaded = append(uploaded, path)
		lock.Unlock()
		return nil
	}
	configPath := "robots/configs/fake_data_manager.json"
	testCfg := setupConfig(t, configPath)
	err := setConfigIntervalMins(testCfg, 0.0041)
	test.That(t, err, test.ShouldBeNil)

	// Make the captureDir where we're logging data for our arm.
	captureDir := "/tmp/capture"
	armDir := captureDir + "/arm/arm1/GetEndPosition"
	defer resetFolder(t, armDir)

	// Initialize the data manager and update it with our config.
	dmsvc := newTestDataManager(t, "arm1", "")
	dmsvc.SetUploadFn(uploadFn)
	dmsvc.Update(context.TODO(), testCfg)

	// We set sync_interval_mins to be about 250ms in the config, so wait 150ms so data is captured but not synced.
	time.Sleep(time.Millisecond * 150)

	// Simulate turning off the service.
	err = dmsvc.Close(context.TODO())
	test.That(t, err, test.ShouldBeNil)

	// Validate nothing has been synced yet.
	test.That(t, len(uploaded), test.ShouldEqual, 0)

	// Turn the service back on.
	dmsvc = newTestDataManager(t, "arm1", "")
	dmsvc.SetUploadFn(uploadFn)
	dmsvc.Update(context.TODO(), testCfg)

	// Validate that the previously captured file was uploaded at startup.
	time.Sleep(time.Millisecond * 50)
	err = dmsvc.Close(context.TODO())
	test.That(t, err, test.ShouldBeNil)
	test.That(t, len(uploaded), test.ShouldEqual, 1)
}

func setConfigIntervalMins(config *config.Config, interval float64) error {
	svcConfig, ok, err := datamanager.GetServiceConfig(config)
	if err != nil {
		return err
	}
	if !ok {
		return errors.New("failed to get service config")
	}
	svcConfig.SyncIntervalMins = interval
	return nil
}
