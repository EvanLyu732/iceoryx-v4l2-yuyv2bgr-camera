



## Device Info

```bash
$ v4l2-ctl --device /dev/video0 --all
Driver Info:
        Driver name      : tegra-video
        Card type        : vi-output, imx185 30-001b
        Bus info         : platform:tegra-capture-vi:0
        Driver version   : 5.10.104
        Capabilities     : 0x84200001
                Video Capture
                Streaming
                Extended Pix Format
                Device Capabilities
        Device Caps      : 0x04200001
                Video Capture
                Streaming
                Extended Pix Format
Media Driver Info:
        Driver name      : tegra-camrtc-ca
        Model            : NVIDIA Tegra Video Input Device
        Serial           : 
        Bus info         : 
        Media version    : 5.10.104
        Hardware revision: 0x00000003 (3)
        Driver version   : 5.10.104
Interface Info:
        ID               : 0x0300001d
        Type             : V4L Video
Entity Info:
        ID               : 0x0000001b (27)
        Name             : vi-output, imx185 30-001b
        Function         : V4L2 I/O
        Pad 0x0100001c   : 0: Sink
          Link 0x02000021: from remote pad 0x1000003 of entity '13e40000.host1x:nvcsi@15a00000-': Data, Enabled
Priority: 2
Video input : 0 (Camera 0: ok)
Format Video Capture:
        Width/Height      : 1920/1080
        Pixel Format      : 'YUYV' (YUYV 4:2:2)
        Field             : None
        Bytes per Line    : 3840
        Size Image        : 4147200
        Colorspace        : sRGB
        Transfer Function : Default (maps to sRGB)
        YCbCr/HSV Encoding: Default (maps to ITU-R 601)
        Quantization      : Default (maps to Limited Range)
        Flags             : 

Camera Controls

                     group_hold 0x009a2003 (bool)   : default=0 value=0 flags=execute-on-write
                     hdr_enable 0x009a2004 (intmenu): min=0 max=1 default=0 value=0
                                0: 0 (0x0)
                                1: 1 (0x1)
                        fuse_id 0x009a2007 (str)    : min=0 max=12 step=2 value='000000000000' flags=read-only, has-payload
                    sensor_mode 0x009a2008 (int64)  : min=0 max=3 step=1 default=0 value=0 flags=slider
                           gain 0x009a2009 (int64)  : min=0 max=480 step=3 default=0 value=0 flags=slider
                       exposure 0x009a200a (int64)  : min=30 max=660000 step=1 default=33334 value=30 flags=slider
                     frame_rate 0x009a200b (int64)  : min=1500000 max=30000000 step=1 default=30000000 value=1500000 flags=slider
           sensor_configuration 0x009a2032 (u32)    : min=0 max=4294967295 step=1 default=0 [22] flags=read-only, volatile, has-payload
         sensor_mode_i2c_packet 0x009a2033 (u32)    : min=0 max=4294967295 step=1 default=0 [1026] flags=read-only, volatile, has-payload
      sensor_control_i2c_packet 0x009a2034 (u32)    : min=0 max=4294967295 step=1 default=0 [1026] flags=read-only, volatile, has-payload
                    bypass_mode 0x009a2064 (intmenu): min=0 max=1 default=0 value=0
                                0: 0 (0x0)
                                1: 1 (0x1)
                override_enable 0x009a2065 (intmenu): min=0 max=1 default=0 value=0
                                0: 0 (0x0)
                                1: 1 (0x1)
                   height_align 0x009a2066 (int)    : min=1 max=16 step=1 default=1 value=1
                     size_align 0x009a2067 (intmenu): min=0 max=2 default=0 value=0
                                0: 1 (0x1)
                                1: 65536 (0x10000)
                                2: 131072 (0x20000)
               write_isp_format 0x009a2068 (int)    : min=1 max=1 step=1 default=1 value=1
       sensor_signal_properties 0x009a2069 (u32)    : min=0 max=4294967295 step=1 default=0 [30][18] flags=read-only, has-payload
        sensor_image_properties 0x009a206a (u32)    : min=0 max=4294967295 step=1 default=0 [30][16] flags=read-only, has-payload
      sensor_control_properties 0x009a206b (u32)    : min=0 max=4294967295 step=1 default=0 [30][36] flags=read-only, has-payload
              sensor_dv_timings 0x009a206c (u32)    : min=0 max=4294967295 step=1 default=0 [30][16] flags=read-only, has-payload
               low_latency_mode 0x009a206d (bool)   : default=0 value=0
               preferred_stride 0x009a206e (int)    : min=0 max=65535 step=1 default=0 value=0
                   sensor_modes 0x009a2082 (int)    : min=0 max=30 step=1 default=30 value=3 flags=read-only
```


## Traps

- using `v4l2_open` api would lead to unexpected error.