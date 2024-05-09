// Package state controls the source of the RTP packets being written to the stream's subscribers
// and ensures there is only one active at a time while there are peer connections to receive RTP packets.
package state

import (
	"context"
	"fmt"
	"sync"
	"sync/atomic"
	"time"

	"github.com/pkg/errors"
	"go.uber.org/multierr"
	"go.viam.com/utils"

	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/gostream"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/robot"
)

var (
	subscribeRTPTimeout = time.Second * 5
	// UnsubscribeTimeout is the timeout used when unsubscribing from an rtppassthrough subscription.
	UnsubscribeTimeout = time.Second * 5
	// ErrRTPPassthroughNotSupported indicates that rtp_passthrough is not supported by the stream's camera.
	ErrRTPPassthroughNotSupported = errors.New("RTP Passthrough Not Supported")
	// ErrClosed indicates that the StreamState is already closed.
	ErrClosed = errors.New("StreamState already closed")
	// ErrUninitialized indicates that Init() has not been called on StreamState prior to Increment or Decrement being called.
	ErrUninitialized = errors.New("uniniialized")
)

// StreamState controls the source of the RTP packets being written to the stream's subscribers
// and ensures there is only one active at a time while there are subsribers.
type StreamState struct {
	// Stream is the StreamState's stream
	Stream      gostream.Stream
	robot       robot.Robot
	closedCtx   context.Context
	closedFn    context.CancelFunc
	wg          sync.WaitGroup
	logger      logging.Logger
	initialized atomic.Bool

	msgChan     chan msg
	restartChan chan struct{}

	activePeers  int
	streamSource streamSource
}

// New returns a new *StreamState.
// rtpPassthroughSource is allowed to be nil
// if the camere does not implement rtppassthrough.Source.
func New(
	stream gostream.Stream,
	r robot.Robot,
	logger logging.Logger,
) *StreamState {
	ctx, cancel := context.WithCancel(context.Background())
	return &StreamState{
		Stream:      stream,
		closedCtx:   ctx,
		closedFn:    cancel,
		robot:       r,
		msgChan:     make(chan msg),
		restartChan: make(chan struct{}),
		logger:      logger,
	}
}

// Init initializes the StreamState
// Init must be called before any other methods.
func (ss *StreamState) Init() {
	ss.wg.Add(1)
	utils.ManagedGo(ss.initEventHandler, ss.wg.Done)
	ss.wg.Add(1)
	utils.ManagedGo(ss.initStreamSourceMonitor, ss.wg.Done)
	ss.initialized.Store(true)
}

// Increment increments the peer connections subscribed to the stream.
func (ss *StreamState) Increment(ctx context.Context) error {
	if !ss.initialized.Load() {
		return ErrUninitialized
	}
	if err := ss.closedCtx.Err(); err != nil {
		return multierr.Combine(ErrClosed, err)
	}
	return ss.send(ctx, msgTypeIncrement)
}

// Decrement decrements the peer connections subscribed to the stream.
func (ss *StreamState) Decrement(ctx context.Context) error {
	if !ss.initialized.Load() {
		return ErrUninitialized
	}
	if err := ss.closedCtx.Err(); err != nil {
		return multierr.Combine(ErrClosed, err)
	}
	return ss.send(ctx, msgTypeDecrement)
}

// Restart restarts the stream source after it has terminated.
func (ss *StreamState) Restart(ctx context.Context) {
	if err := ss.closedCtx.Err(); err != nil {
		return
	}
	utils.UncheckedError(ss.send(ctx, msgTypeRestart))
}

// Close closes the StreamState.
func (ss *StreamState) Close() error {
	ss.closedFn()
	ss.wg.Wait()
	return nil
}

// Internals

const rtpBufferSize int = 512

type streamSource uint8

const (
	streamSourceUnknown streamSource = iota
	streamSourceGoStream
	streamSourcePassthrough
)

func (s streamSource) String() string {
	switch s {
	case streamSourceGoStream:
		return "GoStream"
	case streamSourcePassthrough:
		return "RTP Passthrough"
	case streamSourceUnknown:
		fallthrough
	default:
		return "Unknown"
	}
}

type msgType uint8

const (
	msgTypeUnknown msgType = iota
	msgTypeIncrement
	msgTypeDecrement
	msgTypeRestart
)

func (mt msgType) String() string {
	switch mt {
	case msgTypeIncrement:
		return "Increment"
	case msgTypeDecrement:
		return "Decrement"
	case msgTypeRestart:
		return "Restart"
	case msgTypeUnknown:
		fallthrough
	default:
		return "Unknown"
	}
}

type msg struct {
	msgType  msgType
	ctx      context.Context
	respChan chan error
}

// events (Inc Dec Restart).
func (ss *StreamState) initEventHandler() {
	ss.logger.Debug("StreamState initEventHandler booted")
	defer ss.logger.Debug("StreamState initEventHandler terminated")
	defer func() {
		ctx, cancel := context.WithTimeout(context.Background(), UnsubscribeTimeout)
		defer cancel()
		utils.UncheckedError(ss.stopBasedOnSub(ctx))
	}()
	for {
		if ss.closedCtx.Err() != nil {
			return
		}

		select {
		case <-ss.closedCtx.Done():
			return
		case msg := <-ss.msgChan:
			ss.handleMsg(msg)
		}
	}
}

func (ss *StreamState) initStreamSourceMonitor() {
	for {
		select {
		case <-ss.closedCtx.Done():
			return
		case <-ss.restartChan:
			ctx, cancel := context.WithTimeout(ss.closedCtx, subscribeRTPTimeout)
			ss.Restart(ctx)
			cancel()
		}
	}
}

// caller must be holding ss.mu.
func (ss *StreamState) stopBasedOnSub(ctx context.Context) error {
	switch ss.streamSource {
	case streamSourceGoStream:
		ss.logger.Debugf("%s stopBasedOnSub stopping GoStream", ss.Stream.Name())
		ss.Stream.Stop()
		ss.streamSource = streamSourceUnknown
		return nil
	case streamSourcePassthrough:
		return nil

	case streamSourceUnknown:
		fallthrough
	default:
		return nil
	}
}

func (ss *StreamState) send(ctx context.Context, msgType msgType) error {
	if err := ctx.Err(); err != nil {
		return err
	}
	if err := ss.closedCtx.Err(); err != nil {
		return err
	}
	msg := msg{
		ctx:      ctx,
		msgType:  msgType,
		respChan: make(chan error),
	}
	select {
	case ss.msgChan <- msg:
		select {
		case err := <-msg.respChan:
			return err
		case <-ctx.Done():
			return ctx.Err()
		case <-ss.closedCtx.Done():
			return ss.closedCtx.Err()
		}
	case <-ctx.Done():
		return ctx.Err()
	case <-ss.closedCtx.Done():
		return ss.closedCtx.Err()
	}
}

func (ss *StreamState) handleMsg(msg msg) {
	switch msg.msgType {
	case msgTypeIncrement:
		err := ss.inc(msg.ctx)
		select {
		case msg.respChan <- err:
		case <-ss.closedCtx.Done():
			return
		}
	case msgTypeRestart:
		ss.restart(msg.ctx)
		select {
		case msg.respChan <- nil:
		case <-ss.closedCtx.Done():
			return
		}
	case msgTypeDecrement:
		err := ss.dec(msg.ctx)
		msg.respChan <- err
	case msgTypeUnknown:
		fallthrough
	default:
		ss.logger.Error("Invalid StreamState msg type received: %s", msg.msgType)
	}
}

func (ss *StreamState) inc(ctx context.Context) error {
	ss.logger.Debugf("increment %s START activePeers: %d", ss.Stream.Name(), ss.activePeers)
	defer func() { ss.logger.Debugf("increment %s END activePeers: %d", ss.Stream.Name(), ss.activePeers) }()
	if ss.activePeers == 0 {
		if ss.streamSource != streamSourceUnknown {
			return fmt.Errorf("unexpected stream %s source %s", ss.Stream.Name(), ss.streamSource)
		}
		// this is the first subscription, attempt passthrough
		ss.logger.CDebugw(ctx, "attempting to subscribe to rtp_passthrough", "name", ss.Stream.Name())
		err := ss.streamH264Passthrough(ctx)
		if err != nil {
			ss.logger.CDebugw(ctx, "rtp_passthrough not possible, falling back to GoStream", "err", err.Error(), "name", ss.Stream.Name())
			// if passthrough failed, fall back to gostream based approach
			ss.Stream.Start()
			ss.streamSource = streamSourceGoStream
		}
		ss.activePeers++
		return nil
	}

	switch ss.streamSource {
	case streamSourcePassthrough:
		ss.logger.CDebugw(ctx, "continuing using rtp_passthrough", "name", ss.Stream.Name())
		// noop as we are already subscribed
	case streamSourceGoStream:
		ss.logger.CDebugw(ctx, "currently using gostream, trying upgrade to rtp_passthrough", "name", ss.Stream.Name())
		// attempt to cut over to passthrough
		err := ss.streamH264Passthrough(ctx)
		if err != nil {
			ss.logger.CDebugw(ctx, "rtp_passthrough not possible, continuing with gostream", "err", err.Error(), "name", ss.Stream.Name())
		}
	case streamSourceUnknown:
		fallthrough
	default:
		err := fmt.Errorf("%s streamSource in unexpected state %s", ss.Stream.Name(), ss.streamSource)
		ss.logger.Error(err.Error())
		return err
	}
	ss.activePeers++
	return nil
}

func (ss *StreamState) dec(ctx context.Context) error {
	ss.logger.Debugf("decrement START %s activePeers: %d", ss.Stream.Name(), ss.activePeers)
	defer func() { ss.logger.Debugf("decrement END %s activePeers: %d", ss.Stream.Name(), ss.activePeers) }()

	var err error
	defer func() {
		if err != nil {
			ss.logger.Errorf("decrement %s hit error: %s", ss.Stream.Name(), err.Error())
			return
		}
		ss.activePeers--
		if ss.activePeers <= 0 {
			ss.activePeers = 0
		}
	}()
	if ss.activePeers == 1 {
		ss.logger.Debugf("decrement %s calling stopBasedOnSub", ss.Stream.Name())
		err = ss.stopBasedOnSub(ctx)
		if err != nil {
			ss.logger.Error(err.Error())
			return err
		}
	}
	return nil
}

func (ss *StreamState) restart(ctx context.Context) {
	ss.logger.Debugf("restart %s START activePeers: %d", ss.Stream.Name(), ss.activePeers)
	defer func() { ss.logger.Debugf("restart %s END activePeers: %d", ss.Stream.Name(), ss.activePeers) }()

	if ss.activePeers == 0 {
		// nothing to do if we don't have any active peers
		return
	}

	if ss.streamSource == streamSourceGoStream {
		// nothing to do if stream source is gostream
		return
	}

	err := ss.streamH264Passthrough(ctx)
	if err != nil {
		ss.logger.CDebugw(ctx, "rtp_passthrough not possible, falling back to GoStream", "err", err.Error(), "name", ss.Stream.Name())
		// if passthrough failed, fall back to gostream based approach
		ss.Stream.Start()
		ss.streamSource = streamSourceGoStream

		return
	}
	// passthrough succeeded, listen for when subscription end and call start again if so
}

func (ss *StreamState) streamH264Passthrough(ctx context.Context) error {
	_, err := camera.FromRobot(ss.robot, ss.Stream.Name())
	if err != nil {
		return err
	}

	return nil
}
