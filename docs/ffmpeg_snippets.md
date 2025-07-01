## Extract left rgb from SpatialMP4

```
ffmpeg -i 3DVideo_2025-06-18-20-53-26-759.mp4 -vf "crop=iw/2:ih:0:0" -q:v 2 left_%08d.png
```
