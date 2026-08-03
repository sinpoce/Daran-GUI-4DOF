# 正式三维 ChArUco 标定板

这是机械臂 GUI 唯一支持的正式标定板。标定板包含三个不同法向的表面，共 42 个全局角点。

## 文件

- `multi_plane_target.stl`：三维打印件，单位为毫米。
- `strips_A4.pdf`：三条 300 DPI 图案，必须按 100% 比例打印。
- `target_model.json`：标定求解和测试点使用的三维角点模型。
- `target_preview.png`：三维结构预览。
- `strips_A4_preview.png`：打印页预览。

## 制作要求

1. STL 底面朝下打印，禁止缩放，推荐 0.12 mm 层高、至少四道壁、20% 以上填充。
2. 哑光纸按 100% 比例打印 `strips_A4.pdf`，关闭“适应页面”。
3. 按 L0、L1、L2 标签粘贴到对应表面，打印页的上边保持朝向标定板 +Y。
4. 标定及测试过程中不得移动标定板。

## ID 约定

- L0 marker：0–11；L1 marker：12–23；L2 marker：24–35。
- 每条带局部 ChArUco corner ID：0–13。
- 全局角点 ID：`level * 14 + local_id`，范围 0–41。

模型中的 `measured_geometry` 为空时使用CAD设计尺寸，绝对精度会受到打印和粘贴公差限制。
