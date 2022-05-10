// Code generated by protoc-gen-go-grpc. DO NOT EDIT.

package v1

import (
	context "context"
	grpc "google.golang.org/grpc"
	codes "google.golang.org/grpc/codes"
	status "google.golang.org/grpc/status"
)

// This is a compile-time assertion to ensure that this generated file
// is compatible with the grpc package it is being compiled against.
// Requires gRPC-Go v1.32.0 or later.
const _ = grpc.SupportPackageIsVersion7

// BaseServiceClient is the client API for BaseService service.
//
// For semantics around ctx use and closing/ending streaming RPCs, please refer to https://pkg.go.dev/google.golang.org/grpc/?tab=doc#ClientConn.NewStream.
type BaseServiceClient interface {
	// MoveStraight moves a robot's base in a straight line by a given distance, expressed in millimeters
	// and a given speed, expressed in millimeters per second
	// This method blocks until completed or cancelled
	MoveStraight(ctx context.Context, in *MoveStraightRequest, opts ...grpc.CallOption) (*MoveStraightResponse, error)
	// MoveArc moves the robot's base in an arc by a given distance, expressed in millimeters,
	// a given speed, expressed in millimeters per second of movement, and a given angle expressed in degrees
	// This method blocks until completed or cancelled
	MoveArc(ctx context.Context, in *MoveArcRequest, opts ...grpc.CallOption) (*MoveArcResponse, error)
	// Spin spins a robot's base by an given angle, expressed in degrees, and a given
	// angular speed, expressed in degrees per second
	// This method blocks until completed or cancelled
	Spin(ctx context.Context, in *SpinRequest, opts ...grpc.CallOption) (*SpinResponse, error)
	// Stop stops a robot's base
	Stop(ctx context.Context, in *StopRequest, opts ...grpc.CallOption) (*StopResponse, error)
}

type baseServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewBaseServiceClient(cc grpc.ClientConnInterface) BaseServiceClient {
	return &baseServiceClient{cc}
}

func (c *baseServiceClient) MoveStraight(ctx context.Context, in *MoveStraightRequest, opts ...grpc.CallOption) (*MoveStraightResponse, error) {
	out := new(MoveStraightResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.base.v1.BaseService/MoveStraight", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *baseServiceClient) MoveArc(ctx context.Context, in *MoveArcRequest, opts ...grpc.CallOption) (*MoveArcResponse, error) {
	out := new(MoveArcResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.base.v1.BaseService/MoveArc", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *baseServiceClient) Spin(ctx context.Context, in *SpinRequest, opts ...grpc.CallOption) (*SpinResponse, error) {
	out := new(SpinResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.base.v1.BaseService/Spin", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

func (c *baseServiceClient) Stop(ctx context.Context, in *StopRequest, opts ...grpc.CallOption) (*StopResponse, error) {
	out := new(StopResponse)
	err := c.cc.Invoke(ctx, "/proto.api.component.base.v1.BaseService/Stop", in, out, opts...)
	if err != nil {
		return nil, err
	}
	return out, nil
}

// BaseServiceServer is the server API for BaseService service.
// All implementations must embed UnimplementedBaseServiceServer
// for forward compatibility
type BaseServiceServer interface {
	// MoveStraight moves a robot's base in a straight line by a given distance, expressed in millimeters
	// and a given speed, expressed in millimeters per second
	// This method blocks until completed or cancelled
	MoveStraight(context.Context, *MoveStraightRequest) (*MoveStraightResponse, error)
	// MoveArc moves the robot's base in an arc by a given distance, expressed in millimeters,
	// a given speed, expressed in millimeters per second of movement, and a given angle expressed in degrees
	// This method blocks until completed or cancelled
	MoveArc(context.Context, *MoveArcRequest) (*MoveArcResponse, error)
	// Spin spins a robot's base by an given angle, expressed in degrees, and a given
	// angular speed, expressed in degrees per second
	// This method blocks until completed or cancelled
	Spin(context.Context, *SpinRequest) (*SpinResponse, error)
	// Stop stops a robot's base
	Stop(context.Context, *StopRequest) (*StopResponse, error)
	mustEmbedUnimplementedBaseServiceServer()
}

// UnimplementedBaseServiceServer must be embedded to have forward compatible implementations.
type UnimplementedBaseServiceServer struct {
}

func (UnimplementedBaseServiceServer) MoveStraight(context.Context, *MoveStraightRequest) (*MoveStraightResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method MoveStraight not implemented")
}
func (UnimplementedBaseServiceServer) MoveArc(context.Context, *MoveArcRequest) (*MoveArcResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method MoveArc not implemented")
}
func (UnimplementedBaseServiceServer) Spin(context.Context, *SpinRequest) (*SpinResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method Spin not implemented")
}
func (UnimplementedBaseServiceServer) Stop(context.Context, *StopRequest) (*StopResponse, error) {
	return nil, status.Errorf(codes.Unimplemented, "method Stop not implemented")
}
func (UnimplementedBaseServiceServer) mustEmbedUnimplementedBaseServiceServer() {}

// UnsafeBaseServiceServer may be embedded to opt out of forward compatibility for this service.
// Use of this interface is not recommended, as added methods to BaseServiceServer will
// result in compilation errors.
type UnsafeBaseServiceServer interface {
	mustEmbedUnimplementedBaseServiceServer()
}

func RegisterBaseServiceServer(s grpc.ServiceRegistrar, srv BaseServiceServer) {
	s.RegisterService(&BaseService_ServiceDesc, srv)
}

func _BaseService_MoveStraight_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(MoveStraightRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BaseServiceServer).MoveStraight(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.base.v1.BaseService/MoveStraight",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BaseServiceServer).MoveStraight(ctx, req.(*MoveStraightRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BaseService_MoveArc_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(MoveArcRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BaseServiceServer).MoveArc(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.base.v1.BaseService/MoveArc",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BaseServiceServer).MoveArc(ctx, req.(*MoveArcRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BaseService_Spin_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(SpinRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BaseServiceServer).Spin(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.base.v1.BaseService/Spin",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BaseServiceServer).Spin(ctx, req.(*SpinRequest))
	}
	return interceptor(ctx, in, info, handler)
}

func _BaseService_Stop_Handler(srv interface{}, ctx context.Context, dec func(interface{}) error, interceptor grpc.UnaryServerInterceptor) (interface{}, error) {
	in := new(StopRequest)
	if err := dec(in); err != nil {
		return nil, err
	}
	if interceptor == nil {
		return srv.(BaseServiceServer).Stop(ctx, in)
	}
	info := &grpc.UnaryServerInfo{
		Server:     srv,
		FullMethod: "/proto.api.component.base.v1.BaseService/Stop",
	}
	handler := func(ctx context.Context, req interface{}) (interface{}, error) {
		return srv.(BaseServiceServer).Stop(ctx, req.(*StopRequest))
	}
	return interceptor(ctx, in, info, handler)
}

// BaseService_ServiceDesc is the grpc.ServiceDesc for BaseService service.
// It's only intended for direct use with grpc.RegisterService,
// and not to be introspected or modified (even as a copy)
var BaseService_ServiceDesc = grpc.ServiceDesc{
	ServiceName: "proto.api.component.base.v1.BaseService",
	HandlerType: (*BaseServiceServer)(nil),
	Methods: []grpc.MethodDesc{
		{
			MethodName: "MoveStraight",
			Handler:    _BaseService_MoveStraight_Handler,
		},
		{
			MethodName: "MoveArc",
			Handler:    _BaseService_MoveArc_Handler,
		},
		{
			MethodName: "Spin",
			Handler:    _BaseService_Spin_Handler,
		},
		{
			MethodName: "Stop",
			Handler:    _BaseService_Stop_Handler,
		},
	},
	Streams:  []grpc.StreamDesc{},
	Metadata: "proto/api/component/base/v1/base.proto",
}
