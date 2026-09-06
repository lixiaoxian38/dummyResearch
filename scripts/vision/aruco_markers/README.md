# ArUco 标定板（A4 打印）

## 直接打印

**文件：** [`aruco_original_id0_50mm_a4.pdf`](aruco_original_id0_50mm_a4.pdf)

| 项 | 值 |
|---|---|
| 字典 | DICT_ARUCO_ORIGINAL |
| Marker ID | 0 |
| 黑色方块边长 | **50 mm** |
| ROS 参数 | `marker_length:=0.05` `marker_id:=0` |

### 打印设置

1. 用 PDF 阅读器打开上述文件  
2. 选 **100%** / **实际大小**（不要「适合页面」）  
3. 打印后 **用尺子量黑色方块**，必须是 **5.0 cm**  
4. 贴硬纸板，尽量平整

### 重新生成

```bash
python3 scripts/vision/generate_aruco_a4_pdf.py
# 可选: --marker-id 0 --marker-mm 50 --dict ORIGINAL
```

## 旧版 PNG

- `aruco_original_id0.png` — 小图，未按 A4 物理尺寸排版  
- `aruco_original_2x2_ids0-3.png` — 四格测试用

正式标定请用 **PDF**。
