// Code generated by protoc-gen-go. DO NOT EDIT.
// versions:
// 	protoc-gen-go v1.27.1
// 	protoc        v3.19.1
// source: proto/api/component/v1/arm.proto

package v1

import (
	reflect "reflect"
	sync "sync"

	_ "google.golang.org/genproto/googleapis/api/annotations"
	_ "google.golang.org/genproto/googleapis/api/httpbody"
	protoreflect "google.golang.org/protobuf/reflect/protoreflect"
	protoimpl "google.golang.org/protobuf/runtime/protoimpl"
	_ "google.golang.org/protobuf/types/known/durationpb"
	_ "google.golang.org/protobuf/types/known/structpb"

	v1 "go.viam.com/core/proto/api/common/v1"
)

const (
	// Verify that this generated code is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(20 - protoimpl.MinVersion)
	// Verify that runtime/protoimpl is sufficiently up-to-date.
	_ = protoimpl.EnforceVersion(protoimpl.MaxVersion - 20)
)

type ArmJointPositions struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// A list of joint positions represented in degrees
	// The numbers are ordered spatially from the base toward the end effector
	// This is used in ArmServiceCurrentJointPositionsResponse and ArmServiceMoveToJointPositionsRequest
	Degrees []float64 `protobuf:"fixed64,1,rep,packed,name=degrees,proto3" json:"degrees,omitempty"`
}

func (x *ArmJointPositions) Reset() {
	*x = ArmJointPositions{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[0]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmJointPositions) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmJointPositions) ProtoMessage() {}

func (x *ArmJointPositions) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[0]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmJointPositions.ProtoReflect.Descriptor instead.
func (*ArmJointPositions) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{0}
}

func (x *ArmJointPositions) GetDegrees() []float64 {
	if x != nil {
		return x.Degrees
	}
	return nil
}

type ArmServiceCurrentPositionRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// Name of an arm
	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
}

func (x *ArmServiceCurrentPositionRequest) Reset() {
	*x = ArmServiceCurrentPositionRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[1]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceCurrentPositionRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceCurrentPositionRequest) ProtoMessage() {}

func (x *ArmServiceCurrentPositionRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[1]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceCurrentPositionRequest.ProtoReflect.Descriptor instead.
func (*ArmServiceCurrentPositionRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{1}
}

func (x *ArmServiceCurrentPositionRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

type ArmServiceCurrentPositionResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// Returns 6d pose of the end effector relative to the base, represented by X,Y,Z coordinates which express
	// millimeters and theta, ox, oy, oz coordinates which express an orientation vector
	Position *v1.Pose `protobuf:"bytes,1,opt,name=position,proto3" json:"position,omitempty"`
}

func (x *ArmServiceCurrentPositionResponse) Reset() {
	*x = ArmServiceCurrentPositionResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[2]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceCurrentPositionResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceCurrentPositionResponse) ProtoMessage() {}

func (x *ArmServiceCurrentPositionResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[2]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceCurrentPositionResponse.ProtoReflect.Descriptor instead.
func (*ArmServiceCurrentPositionResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{2}
}

func (x *ArmServiceCurrentPositionResponse) GetPosition() *v1.Pose {
	if x != nil {
		return x.Position
	}
	return nil
}

type ArmServiceCurrentJointPositionsRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// Name of an arm
	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
}

func (x *ArmServiceCurrentJointPositionsRequest) Reset() {
	*x = ArmServiceCurrentJointPositionsRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[3]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceCurrentJointPositionsRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceCurrentJointPositionsRequest) ProtoMessage() {}

func (x *ArmServiceCurrentJointPositionsRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[3]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceCurrentJointPositionsRequest.ProtoReflect.Descriptor instead.
func (*ArmServiceCurrentJointPositionsRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{3}
}

func (x *ArmServiceCurrentJointPositionsRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

type ArmServiceCurrentJointPositionsResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	//a list ArmJointPositions
	Positions *ArmJointPositions `protobuf:"bytes,1,opt,name=positions,proto3" json:"positions,omitempty"`
}

func (x *ArmServiceCurrentJointPositionsResponse) Reset() {
	*x = ArmServiceCurrentJointPositionsResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[4]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceCurrentJointPositionsResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceCurrentJointPositionsResponse) ProtoMessage() {}

func (x *ArmServiceCurrentJointPositionsResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[4]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceCurrentJointPositionsResponse.ProtoReflect.Descriptor instead.
func (*ArmServiceCurrentJointPositionsResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{4}
}

func (x *ArmServiceCurrentJointPositionsResponse) GetPositions() *ArmJointPositions {
	if x != nil {
		return x.Positions
	}
	return nil
}

type ArmServiceMoveToPositionRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// Name of an arm
	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
	// X, Y, Z, ox, oy, ox, theta
	To *v1.Pose `protobuf:"bytes,2,opt,name=to,proto3" json:"to,omitempty"`
}

func (x *ArmServiceMoveToPositionRequest) Reset() {
	*x = ArmServiceMoveToPositionRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[5]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceMoveToPositionRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceMoveToPositionRequest) ProtoMessage() {}

func (x *ArmServiceMoveToPositionRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[5]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceMoveToPositionRequest.ProtoReflect.Descriptor instead.
func (*ArmServiceMoveToPositionRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{5}
}

func (x *ArmServiceMoveToPositionRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

func (x *ArmServiceMoveToPositionRequest) GetTo() *v1.Pose {
	if x != nil {
		return x.To
	}
	return nil
}

type ArmServiceMoveToPositionResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields
}

func (x *ArmServiceMoveToPositionResponse) Reset() {
	*x = ArmServiceMoveToPositionResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[6]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceMoveToPositionResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceMoveToPositionResponse) ProtoMessage() {}

func (x *ArmServiceMoveToPositionResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[6]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceMoveToPositionResponse.ProtoReflect.Descriptor instead.
func (*ArmServiceMoveToPositionResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{6}
}

type ArmServiceMoveToJointPositionsRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// Name of an arm
	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
	// A list of joint positions represented in degrees
	// There should be 1 entry in the list per joint, ordered spatially from the base toward the end effector
	To *ArmJointPositions `protobuf:"bytes,2,opt,name=to,proto3" json:"to,omitempty"`
}

func (x *ArmServiceMoveToJointPositionsRequest) Reset() {
	*x = ArmServiceMoveToJointPositionsRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[7]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceMoveToJointPositionsRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceMoveToJointPositionsRequest) ProtoMessage() {}

func (x *ArmServiceMoveToJointPositionsRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[7]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceMoveToJointPositionsRequest.ProtoReflect.Descriptor instead.
func (*ArmServiceMoveToJointPositionsRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{7}
}

func (x *ArmServiceMoveToJointPositionsRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

func (x *ArmServiceMoveToJointPositionsRequest) GetTo() *ArmJointPositions {
	if x != nil {
		return x.To
	}
	return nil
}

type ArmServiceMoveToJointPositionsResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields
}

func (x *ArmServiceMoveToJointPositionsResponse) Reset() {
	*x = ArmServiceMoveToJointPositionsResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[8]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceMoveToJointPositionsResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceMoveToJointPositionsResponse) ProtoMessage() {}

func (x *ArmServiceMoveToJointPositionsResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[8]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceMoveToJointPositionsResponse.ProtoReflect.Descriptor instead.
func (*ArmServiceMoveToJointPositionsResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{8}
}

type ArmServiceJointMoveDeltaRequest struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields

	// Name of an arm
	Name string `protobuf:"bytes,1,opt,name=name,proto3" json:"name,omitempty"`
	// To specify a joint, refer to base as 0 and count forwards up to the end effector
	Joint int32 `protobuf:"varint,2,opt,name=joint,proto3" json:"joint,omitempty"`
	// Specify degrees, can be positive or negative
	AmountDegs float64 `protobuf:"fixed64,3,opt,name=amount_degs,json=amountDegs,proto3" json:"amount_degs,omitempty"`
}

func (x *ArmServiceJointMoveDeltaRequest) Reset() {
	*x = ArmServiceJointMoveDeltaRequest{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[9]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceJointMoveDeltaRequest) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceJointMoveDeltaRequest) ProtoMessage() {}

func (x *ArmServiceJointMoveDeltaRequest) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[9]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceJointMoveDeltaRequest.ProtoReflect.Descriptor instead.
func (*ArmServiceJointMoveDeltaRequest) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{9}
}

func (x *ArmServiceJointMoveDeltaRequest) GetName() string {
	if x != nil {
		return x.Name
	}
	return ""
}

func (x *ArmServiceJointMoveDeltaRequest) GetJoint() int32 {
	if x != nil {
		return x.Joint
	}
	return 0
}

func (x *ArmServiceJointMoveDeltaRequest) GetAmountDegs() float64 {
	if x != nil {
		return x.AmountDegs
	}
	return 0
}

type ArmServiceJointMoveDeltaResponse struct {
	state         protoimpl.MessageState
	sizeCache     protoimpl.SizeCache
	unknownFields protoimpl.UnknownFields
}

func (x *ArmServiceJointMoveDeltaResponse) Reset() {
	*x = ArmServiceJointMoveDeltaResponse{}
	if protoimpl.UnsafeEnabled {
		mi := &file_proto_api_component_v1_arm_proto_msgTypes[10]
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		ms.StoreMessageInfo(mi)
	}
}

func (x *ArmServiceJointMoveDeltaResponse) String() string {
	return protoimpl.X.MessageStringOf(x)
}

func (*ArmServiceJointMoveDeltaResponse) ProtoMessage() {}

func (x *ArmServiceJointMoveDeltaResponse) ProtoReflect() protoreflect.Message {
	mi := &file_proto_api_component_v1_arm_proto_msgTypes[10]
	if protoimpl.UnsafeEnabled && x != nil {
		ms := protoimpl.X.MessageStateOf(protoimpl.Pointer(x))
		if ms.LoadMessageInfo() == nil {
			ms.StoreMessageInfo(mi)
		}
		return ms
	}
	return mi.MessageOf(x)
}

// Deprecated: Use ArmServiceJointMoveDeltaResponse.ProtoReflect.Descriptor instead.
func (*ArmServiceJointMoveDeltaResponse) Descriptor() ([]byte, []int) {
	return file_proto_api_component_v1_arm_proto_rawDescGZIP(), []int{10}
}

var File_proto_api_component_v1_arm_proto protoreflect.FileDescriptor

var file_proto_api_component_v1_arm_proto_rawDesc = []byte{
	0x0a, 0x20, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x63, 0x6f, 0x6d, 0x70,
	0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x76, 0x31, 0x2f, 0x61, 0x72, 0x6d, 0x2e, 0x70, 0x72, 0x6f,
	0x74, 0x6f, 0x12, 0x16, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f,
	0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x1a, 0x1c, 0x67, 0x6f, 0x6f, 0x67,
	0x6c, 0x65, 0x2f, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x62, 0x75, 0x66, 0x2f, 0x73, 0x74, 0x72, 0x75,
	0x63, 0x74, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x1a, 0x1e, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65,
	0x2f, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x62, 0x75, 0x66, 0x2f, 0x64, 0x75, 0x72, 0x61, 0x74, 0x69,
	0x6f, 0x6e, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x1a, 0x1c, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65,
	0x2f, 0x61, 0x70, 0x69, 0x2f, 0x61, 0x6e, 0x6e, 0x6f, 0x74, 0x61, 0x74, 0x69, 0x6f, 0x6e, 0x73,
	0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x1a, 0x19, 0x67, 0x6f, 0x6f, 0x67, 0x6c, 0x65, 0x2f, 0x61,
	0x70, 0x69, 0x2f, 0x68, 0x74, 0x74, 0x70, 0x62, 0x6f, 0x64, 0x79, 0x2e, 0x70, 0x72, 0x6f, 0x74,
	0x6f, 0x1a, 0x20, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x63, 0x6f, 0x6d,
	0x6d, 0x6f, 0x6e, 0x2f, 0x76, 0x31, 0x2f, 0x63, 0x6f, 0x6d, 0x6d, 0x6f, 0x6e, 0x2e, 0x70, 0x72,
	0x6f, 0x74, 0x6f, 0x22, 0x2d, 0x0a, 0x11, 0x41, 0x72, 0x6d, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50,
	0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x12, 0x18, 0x0a, 0x07, 0x64, 0x65, 0x67, 0x72,
	0x65, 0x65, 0x73, 0x18, 0x01, 0x20, 0x03, 0x28, 0x01, 0x52, 0x07, 0x64, 0x65, 0x67, 0x72, 0x65,
	0x65, 0x73, 0x22, 0x36, 0x0a, 0x20, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65,
	0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52,
	0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01,
	0x20, 0x01, 0x28, 0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x22, 0x5a, 0x0a, 0x21, 0x41, 0x72,
	0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x50,
	0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x12,
	0x35, 0x0a, 0x08, 0x70, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x18, 0x01, 0x20, 0x01, 0x28,
	0x0b, 0x32, 0x19, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f,
	0x6d, 0x6d, 0x6f, 0x6e, 0x2e, 0x76, 0x31, 0x2e, 0x50, 0x6f, 0x73, 0x65, 0x52, 0x08, 0x70, 0x6f,
	0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x22, 0x3c, 0x0a, 0x26, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72,
	0x76, 0x69, 0x63, 0x65, 0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x4a, 0x6f, 0x69, 0x6e, 0x74,
	0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74,
	0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01, 0x20, 0x01, 0x28, 0x09, 0x52, 0x04,
	0x6e, 0x61, 0x6d, 0x65, 0x22, 0x72, 0x0a, 0x27, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69,
	0x63, 0x65, 0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f,
	0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x12,
	0x47, 0x0a, 0x09, 0x70, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x18, 0x01, 0x20, 0x01,
	0x28, 0x0b, 0x32, 0x29, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63,
	0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41, 0x72, 0x6d, 0x4a,
	0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x52, 0x09, 0x70,
	0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x22, 0x60, 0x0a, 0x1f, 0x41, 0x72, 0x6d, 0x53,
	0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x50, 0x6f, 0x73, 0x69,
	0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e,
	0x61, 0x6d, 0x65, 0x18, 0x01, 0x20, 0x01, 0x28, 0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x12,
	0x29, 0x0a, 0x02, 0x74, 0x6f, 0x18, 0x02, 0x20, 0x01, 0x28, 0x0b, 0x32, 0x19, 0x2e, 0x70, 0x72,
	0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x6d, 0x6f, 0x6e, 0x2e, 0x76,
	0x31, 0x2e, 0x50, 0x6f, 0x73, 0x65, 0x52, 0x02, 0x74, 0x6f, 0x22, 0x22, 0x0a, 0x20, 0x41, 0x72,
	0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x50, 0x6f,
	0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x76,
	0x0a, 0x25, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x4d, 0x6f, 0x76, 0x65,
	0x54, 0x6f, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73,
	0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18,
	0x01, 0x20, 0x01, 0x28, 0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x12, 0x39, 0x0a, 0x02, 0x74,
	0x6f, 0x18, 0x02, 0x20, 0x01, 0x28, 0x0b, 0x32, 0x29, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e,
	0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31,
	0x2e, 0x41, 0x72, 0x6d, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f,
	0x6e, 0x73, 0x52, 0x02, 0x74, 0x6f, 0x22, 0x28, 0x0a, 0x26, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72,
	0x76, 0x69, 0x63, 0x65, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50,
	0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65,
	0x22, 0x6c, 0x0a, 0x1f, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x4a, 0x6f,
	0x69, 0x6e, 0x74, 0x4d, 0x6f, 0x76, 0x65, 0x44, 0x65, 0x6c, 0x74, 0x61, 0x52, 0x65, 0x71, 0x75,
	0x65, 0x73, 0x74, 0x12, 0x12, 0x0a, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x18, 0x01, 0x20, 0x01, 0x28,
	0x09, 0x52, 0x04, 0x6e, 0x61, 0x6d, 0x65, 0x12, 0x14, 0x0a, 0x05, 0x6a, 0x6f, 0x69, 0x6e, 0x74,
	0x18, 0x02, 0x20, 0x01, 0x28, 0x05, 0x52, 0x05, 0x6a, 0x6f, 0x69, 0x6e, 0x74, 0x12, 0x1f, 0x0a,
	0x0b, 0x61, 0x6d, 0x6f, 0x75, 0x6e, 0x74, 0x5f, 0x64, 0x65, 0x67, 0x73, 0x18, 0x03, 0x20, 0x01,
	0x28, 0x01, 0x52, 0x0a, 0x61, 0x6d, 0x6f, 0x75, 0x6e, 0x74, 0x44, 0x65, 0x67, 0x73, 0x22, 0x22,
	0x0a, 0x20, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x4a, 0x6f, 0x69, 0x6e,
	0x74, 0x4d, 0x6f, 0x76, 0x65, 0x44, 0x65, 0x6c, 0x74, 0x61, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e,
	0x73, 0x65, 0x32, 0xf5, 0x07, 0x0a, 0x0a, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63,
	0x65, 0x12, 0xbd, 0x01, 0x0a, 0x0f, 0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x50, 0x6f, 0x73,
	0x69, 0x74, 0x69, 0x6f, 0x6e, 0x12, 0x38, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70,
	0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41,
	0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74,
	0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a,
	0x39, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70,
	0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76,
	0x69, 0x63, 0x65, 0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69,
	0x6f, 0x6e, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x35, 0x82, 0xd3, 0xe4, 0x93,
	0x02, 0x2f, 0x12, 0x2d, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31, 0x2f, 0x63, 0x6f, 0x6d, 0x70,
	0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x61, 0x72, 0x6d, 0x2f, 0x7b, 0x6e, 0x61, 0x6d, 0x65, 0x7d,
	0x2f, 0x63, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x5f, 0x70, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f,
	0x6e, 0x12, 0xba, 0x01, 0x0a, 0x0e, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x50, 0x6f, 0x73, 0x69,
	0x74, 0x69, 0x6f, 0x6e, 0x12, 0x37, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69,
	0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41, 0x72,
	0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x50, 0x6f,
	0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a, 0x38, 0x2e,
	0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e,
	0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63,
	0x65, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x52,
	0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x35, 0x82, 0xd3, 0xe4, 0x93, 0x02, 0x2f, 0x1a,
	0x2d, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31, 0x2f, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65,
	0x6e, 0x74, 0x2f, 0x61, 0x72, 0x6d, 0x2f, 0x7b, 0x6e, 0x61, 0x6d, 0x65, 0x7d, 0x2f, 0x6d, 0x6f,
	0x76, 0x65, 0x5f, 0x74, 0x6f, 0x5f, 0x70, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x12, 0xd6,
	0x01, 0x0a, 0x15, 0x43, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50,
	0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x12, 0x3e, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f,
	0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76,
	0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x43, 0x75, 0x72, 0x72,
	0x65, 0x6e, 0x74, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e,
	0x73, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a, 0x3f, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f,
	0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76,
	0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x43, 0x75, 0x72, 0x72,
	0x65, 0x6e, 0x74, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e,
	0x73, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22, 0x3c, 0x82, 0xd3, 0xe4, 0x93, 0x02,
	0x36, 0x12, 0x34, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31, 0x2f, 0x63, 0x6f, 0x6d, 0x70, 0x6f,
	0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x61, 0x72, 0x6d, 0x2f, 0x7b, 0x6e, 0x61, 0x6d, 0x65, 0x7d, 0x2f,
	0x63, 0x75, 0x72, 0x72, 0x65, 0x6e, 0x74, 0x5f, 0x6a, 0x6f, 0x69, 0x6e, 0x74, 0x5f, 0x70, 0x6f,
	0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x12, 0xd3, 0x01, 0x0a, 0x14, 0x4d, 0x6f, 0x76, 0x65,
	0x54, 0x6f, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73,
	0x12, 0x3d, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d,
	0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72,
	0x76, 0x69, 0x63, 0x65, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50,
	0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a,
	0x3e, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70,
	0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76,
	0x69, 0x63, 0x65, 0x4d, 0x6f, 0x76, 0x65, 0x54, 0x6f, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x50, 0x6f,
	0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x52, 0x65, 0x73, 0x70, 0x6f, 0x6e, 0x73, 0x65, 0x22,
	0x3c, 0x82, 0xd3, 0xe4, 0x93, 0x02, 0x36, 0x1a, 0x34, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x76, 0x31,
	0x2f, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x61, 0x72, 0x6d, 0x2f, 0x7b,
	0x6e, 0x61, 0x6d, 0x65, 0x7d, 0x2f, 0x6d, 0x6f, 0x76, 0x65, 0x5f, 0x74, 0x6f, 0x5f, 0x6a, 0x6f,
	0x69, 0x6e, 0x74, 0x5f, 0x70, 0x6f, 0x73, 0x69, 0x74, 0x69, 0x6f, 0x6e, 0x73, 0x12, 0xba, 0x01,
	0x0a, 0x0e, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x4d, 0x6f, 0x76, 0x65, 0x44, 0x65, 0x6c, 0x74, 0x61,
	0x12, 0x37, 0x2e, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d,
	0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e, 0x76, 0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72,
	0x76, 0x69, 0x63, 0x65, 0x4a, 0x6f, 0x69, 0x6e, 0x74, 0x4d, 0x6f, 0x76, 0x65, 0x44, 0x65, 0x6c,
	0x74, 0x61, 0x52, 0x65, 0x71, 0x75, 0x65, 0x73, 0x74, 0x1a, 0x38, 0x2e, 0x70, 0x72, 0x6f, 0x74,
	0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e,
	0x76, 0x31, 0x2e, 0x41, 0x72, 0x6d, 0x53, 0x65, 0x72, 0x76, 0x69, 0x63, 0x65, 0x4a, 0x6f, 0x69,
	0x6e, 0x74, 0x4d, 0x6f, 0x76, 0x65, 0x44, 0x65, 0x6c, 0x74, 0x61, 0x52, 0x65, 0x73, 0x70, 0x6f,
	0x6e, 0x73, 0x65, 0x22, 0x35, 0x82, 0xd3, 0xe4, 0x93, 0x02, 0x2f, 0x1a, 0x2d, 0x2f, 0x61, 0x70,
	0x69, 0x2f, 0x76, 0x31, 0x2f, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x61,
	0x72, 0x6d, 0x2f, 0x7b, 0x6e, 0x61, 0x6d, 0x65, 0x7d, 0x2f, 0x6a, 0x6f, 0x69, 0x6e, 0x74, 0x5f,
	0x6d, 0x6f, 0x76, 0x65, 0x5f, 0x64, 0x65, 0x6c, 0x74, 0x61, 0x42, 0x4f, 0x0a, 0x24, 0x63, 0x6f,
	0x6d, 0x2e, 0x76, 0x69, 0x61, 0x6d, 0x2e, 0x63, 0x6f, 0x72, 0x65, 0x2e, 0x70, 0x72, 0x6f, 0x74,
	0x6f, 0x2e, 0x61, 0x70, 0x69, 0x2e, 0x63, 0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2e,
	0x76, 0x31, 0x5a, 0x27, 0x67, 0x6f, 0x2e, 0x76, 0x69, 0x61, 0x6d, 0x2e, 0x63, 0x6f, 0x6d, 0x2f,
	0x63, 0x6f, 0x72, 0x65, 0x2f, 0x70, 0x72, 0x6f, 0x74, 0x6f, 0x2f, 0x61, 0x70, 0x69, 0x2f, 0x63,
	0x6f, 0x6d, 0x70, 0x6f, 0x6e, 0x65, 0x6e, 0x74, 0x2f, 0x76, 0x31, 0x62, 0x06, 0x70, 0x72, 0x6f,
	0x74, 0x6f, 0x33,
}

var (
	file_proto_api_component_v1_arm_proto_rawDescOnce sync.Once
	file_proto_api_component_v1_arm_proto_rawDescData = file_proto_api_component_v1_arm_proto_rawDesc
)

func file_proto_api_component_v1_arm_proto_rawDescGZIP() []byte {
	file_proto_api_component_v1_arm_proto_rawDescOnce.Do(func() {
		file_proto_api_component_v1_arm_proto_rawDescData = protoimpl.X.CompressGZIP(file_proto_api_component_v1_arm_proto_rawDescData)
	})
	return file_proto_api_component_v1_arm_proto_rawDescData
}

var file_proto_api_component_v1_arm_proto_msgTypes = make([]protoimpl.MessageInfo, 11)
var file_proto_api_component_v1_arm_proto_goTypes = []interface{}{
	(*ArmJointPositions)(nil),                       // 0: proto.api.component.v1.ArmJointPositions
	(*ArmServiceCurrentPositionRequest)(nil),        // 1: proto.api.component.v1.ArmServiceCurrentPositionRequest
	(*ArmServiceCurrentPositionResponse)(nil),       // 2: proto.api.component.v1.ArmServiceCurrentPositionResponse
	(*ArmServiceCurrentJointPositionsRequest)(nil),  // 3: proto.api.component.v1.ArmServiceCurrentJointPositionsRequest
	(*ArmServiceCurrentJointPositionsResponse)(nil), // 4: proto.api.component.v1.ArmServiceCurrentJointPositionsResponse
	(*ArmServiceMoveToPositionRequest)(nil),         // 5: proto.api.component.v1.ArmServiceMoveToPositionRequest
	(*ArmServiceMoveToPositionResponse)(nil),        // 6: proto.api.component.v1.ArmServiceMoveToPositionResponse
	(*ArmServiceMoveToJointPositionsRequest)(nil),   // 7: proto.api.component.v1.ArmServiceMoveToJointPositionsRequest
	(*ArmServiceMoveToJointPositionsResponse)(nil),  // 8: proto.api.component.v1.ArmServiceMoveToJointPositionsResponse
	(*ArmServiceJointMoveDeltaRequest)(nil),         // 9: proto.api.component.v1.ArmServiceJointMoveDeltaRequest
	(*ArmServiceJointMoveDeltaResponse)(nil),        // 10: proto.api.component.v1.ArmServiceJointMoveDeltaResponse
	(*v1.Pose)(nil),                                 // 11: proto.api.common.v1.Pose
}
var file_proto_api_component_v1_arm_proto_depIdxs = []int32{
	11, // 0: proto.api.component.v1.ArmServiceCurrentPositionResponse.position:type_name -> proto.api.common.v1.Pose
	0,  // 1: proto.api.component.v1.ArmServiceCurrentJointPositionsResponse.positions:type_name -> proto.api.component.v1.ArmJointPositions
	11, // 2: proto.api.component.v1.ArmServiceMoveToPositionRequest.to:type_name -> proto.api.common.v1.Pose
	0,  // 3: proto.api.component.v1.ArmServiceMoveToJointPositionsRequest.to:type_name -> proto.api.component.v1.ArmJointPositions
	1,  // 4: proto.api.component.v1.ArmService.CurrentPosition:input_type -> proto.api.component.v1.ArmServiceCurrentPositionRequest
	5,  // 5: proto.api.component.v1.ArmService.MoveToPosition:input_type -> proto.api.component.v1.ArmServiceMoveToPositionRequest
	3,  // 6: proto.api.component.v1.ArmService.CurrentJointPositions:input_type -> proto.api.component.v1.ArmServiceCurrentJointPositionsRequest
	7,  // 7: proto.api.component.v1.ArmService.MoveToJointPositions:input_type -> proto.api.component.v1.ArmServiceMoveToJointPositionsRequest
	9,  // 8: proto.api.component.v1.ArmService.JointMoveDelta:input_type -> proto.api.component.v1.ArmServiceJointMoveDeltaRequest
	2,  // 9: proto.api.component.v1.ArmService.CurrentPosition:output_type -> proto.api.component.v1.ArmServiceCurrentPositionResponse
	6,  // 10: proto.api.component.v1.ArmService.MoveToPosition:output_type -> proto.api.component.v1.ArmServiceMoveToPositionResponse
	4,  // 11: proto.api.component.v1.ArmService.CurrentJointPositions:output_type -> proto.api.component.v1.ArmServiceCurrentJointPositionsResponse
	8,  // 12: proto.api.component.v1.ArmService.MoveToJointPositions:output_type -> proto.api.component.v1.ArmServiceMoveToJointPositionsResponse
	10, // 13: proto.api.component.v1.ArmService.JointMoveDelta:output_type -> proto.api.component.v1.ArmServiceJointMoveDeltaResponse
	9,  // [9:14] is the sub-list for method output_type
	4,  // [4:9] is the sub-list for method input_type
	4,  // [4:4] is the sub-list for extension type_name
	4,  // [4:4] is the sub-list for extension extendee
	0,  // [0:4] is the sub-list for field type_name
}

func init() { file_proto_api_component_v1_arm_proto_init() }
func file_proto_api_component_v1_arm_proto_init() {
	if File_proto_api_component_v1_arm_proto != nil {
		return
	}
	if !protoimpl.UnsafeEnabled {
		file_proto_api_component_v1_arm_proto_msgTypes[0].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmJointPositions); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[1].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceCurrentPositionRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[2].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceCurrentPositionResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[3].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceCurrentJointPositionsRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[4].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceCurrentJointPositionsResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[5].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceMoveToPositionRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[6].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceMoveToPositionResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[7].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceMoveToJointPositionsRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[8].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceMoveToJointPositionsResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[9].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceJointMoveDeltaRequest); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
		file_proto_api_component_v1_arm_proto_msgTypes[10].Exporter = func(v interface{}, i int) interface{} {
			switch v := v.(*ArmServiceJointMoveDeltaResponse); i {
			case 0:
				return &v.state
			case 1:
				return &v.sizeCache
			case 2:
				return &v.unknownFields
			default:
				return nil
			}
		}
	}
	type x struct{}
	out := protoimpl.TypeBuilder{
		File: protoimpl.DescBuilder{
			GoPackagePath: reflect.TypeOf(x{}).PkgPath(),
			RawDescriptor: file_proto_api_component_v1_arm_proto_rawDesc,
			NumEnums:      0,
			NumMessages:   11,
			NumExtensions: 0,
			NumServices:   1,
		},
		GoTypes:           file_proto_api_component_v1_arm_proto_goTypes,
		DependencyIndexes: file_proto_api_component_v1_arm_proto_depIdxs,
		MessageInfos:      file_proto_api_component_v1_arm_proto_msgTypes,
	}.Build()
	File_proto_api_component_v1_arm_proto = out.File
	file_proto_api_component_v1_arm_proto_rawDesc = nil
	file_proto_api_component_v1_arm_proto_goTypes = nil
	file_proto_api_component_v1_arm_proto_depIdxs = nil
}
