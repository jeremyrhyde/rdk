package stream

import (
	"errors"
	"fmt"
	"image"
	"image/draw"
	"log"
	"unsafe"

	"github.com/xlab/libvpx-go/vpx"
)

/*
#cgo pkg-config: vpx
#include <vpx/vpx_encoder.h>

typedef struct GoBytes {
  void *bs;
  int size;
} GoBytesType;

GoBytesType get_frame_buffer(vpx_codec_cx_pkt_t *pkt) {
	// iter has set to NULL when after add new image
	GoBytesType bytes = {NULL, 0};
	if (pkt->kind == VPX_CODEC_CX_FRAME_PKT) {
		bytes.bs = pkt->data.frame.buf;
		bytes.size = pkt->data.frame.sz;
	} else {
		bytes.size = 999;
	}
  return bytes;
}

#include <string.h>
int vpx_img_plane_width(const vpx_image_t *img, int plane) {
  if (plane > 0 && img->x_chroma_shift > 0)
    return (img->d_w + 1) >> img->x_chroma_shift;
  else
    return img->d_w;
}
int vpx_img_plane_height(const vpx_image_t *img, int plane) {
  if (plane > 0 && img->y_chroma_shift > 0)
    return (img->d_h + 1) >> img->y_chroma_shift;
  else
    return img->d_h;
}
int vpx_img_read(vpx_image_t *img, void *bs) {
  int plane;
  for (plane = 0; plane < 3; ++plane) {
    unsigned char *buf = img->planes[plane];
    const int stride = img->stride[plane];
    const int w = vpx_img_plane_width(img, plane) *
                  ((img->fmt & VPX_IMG_FMT_HIGHBITDEPTH) ? 2 : 1);
    const int h = vpx_img_plane_height(img, plane);
    int y;
    for (y = 0; y < h; ++y) {
      memcpy(buf, bs, w);
      // if (fread(buf, 1, w, file) != (size_t)w) return 0;
      buf += stride;
      bs += w;
    }
  }
  return 1;
}
*/
import "C"

type VPXEncoder struct {
	ctx              *vpx.CodecCtx
	iface            *vpx.CodecIface
	allocImg         *vpx.Image
	iter             vpx.CodecIter
	keyFrameInterval int
	frameCount       int
}

type VCodec string

const (
	CodecVP8 VCodec = "V_VP8"
	CodecVP9 VCodec = "V_VP9"
)

func NewVPXEncoder(codec VCodec, width, height int) (*VPXEncoder, error) {
	enc := &VPXEncoder{ctx: vpx.NewCodecCtx()}
	switch codec {
	case CodecVP8:
		enc.iface = vpx.EncoderIfaceVP8()
	case CodecVP9:
		enc.iface = vpx.EncoderIfaceVP9()
	default:
		return nil, fmt.Errorf("[WARN] unsupported VPX codec: %s", codec)
	}

	var cfg vpx.CodecEncCfg
	enc.keyFrameInterval = 1 // MAYBE 5
	err := vpx.Error(vpx.CodecEncConfigDefault(enc.iface, &cfg, 0))
	if err != nil {
		panic(err)
	}
	cfg.Deref()
	cfg.GW = uint32(width)
	cfg.GH = uint32(height)
	cfg.GTimebase = vpx.Rational{
		Num: 1,
		Den: 1000,
	}
	cfg.RcTargetBitrate = 20000000
	cfg.GErrorResilient = 1
	cfg.Free() // free so we get a new one? idk

	err = vpx.Error(vpx.CodecEncInitVer(enc.ctx, enc.iface, &cfg, 0, vpx.EncoderABIVersion))
	if err != nil {
		log.Println("[WARN]", err)
		return enc, nil
	}

	var cImg vpx.Image
	allocImg := vpx.ImageAlloc(&cImg, vpx.ImageFormatI420, uint32(width), uint32(height), 0)
	if allocImg == nil {
		return nil, errors.New("failed to allocate image")
	}
	allocImg.Deref()

	enc.allocImg = allocImg

	return enc, nil
}

func (v *VPXEncoder) Encode(img image.Image) ([]byte, error) {
	var iter vpx.CodecIter // TODO(erd): use the iter in VPXEncoder but right now using it causes "cgo argument has Go pointer to Go pointer"
	iterate := func() ([]byte, error) {
		pkt := vpx.CodecGetCxData(v.ctx, &iter)
		// println("pkt", pkt)
		for pkt != nil {
			pkt.Deref()
			if pkt.Kind == vpx.CodecCxFramePkt {
				// now := time.Now()
				goBytes := C.get_frame_buffer((*C.vpx_codec_cx_pkt_t)(unsafe.Pointer(pkt.Ref())))
				bs := C.GoBytes(goBytes.bs, goBytes.size)
				// fmt.Println("get frame took", time.Since(now))
				// now = time.Now()
				return bs, nil
				// fmt.Println("send chan took", time.Since(now))
			} else {
				// println("not a frame pkt")
			}
			pkt = vpx.CodecGetCxData(v.ctx, &iter)
		}
		return nil, nil
	}
	if v.iter != nil {
		return iterate()
	}

	bounds := img.Bounds()
	imRGBA := image.NewRGBA(image.Rect(0, 0, bounds.Dx(), bounds.Dy()))
	draw.Draw(imRGBA, imRGBA.Bounds(), img, bounds.Min, draw.Src)

	yuvImage := RgbaToYuv(imRGBA)
	C.vpx_img_read((*C.vpx_image_t)(unsafe.Pointer(v.allocImg)), unsafe.Pointer(&yuvImage[0]))

	var flags vpx.EncFrameFlags
	if v.keyFrameInterval > 0 && v.frameCount%v.keyFrameInterval == 0 {
		flags |= 1 // C.VPX_EFLAG_FORCE_KF
	}

	// println("encoding frame", v.frameCount)
	// now := time.Now()
	if err := vpx.Error(vpx.CodecEncode(v.ctx, v.allocImg, vpx.CodecPts(v.frameCount), 1, flags, 0)); err != nil {
		return nil, errors.New(vpx.CodecErrorDetail(v.ctx))
	}
	// fmt.Println("encode time took", time.Since(now))
	v.frameCount++

	v.iter = nil
	return iterate()
}
