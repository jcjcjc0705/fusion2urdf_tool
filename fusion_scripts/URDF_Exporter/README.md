# URDF_Exporter

Fusion 設計 → URDF。本專案使用的版本。

## 安裝

Fusion 裡 `Shift+S` → Scripts 分頁 → 綠色 `+`，參照到：

```
step2urdf/fusion_scripts/URDF_Exporter/URDF_Exporter.py
```

## 匯出前的必要條件

- 每個 link 都是 **root 底下的一層 component**，裡面只有 body，不能有巢狀 component
  （`copy_occs()` 只走 `root.occurrences` 且只抓該層的 `bRepBodies`）
- 底座 component 必須命名為 **`base_link`**，而且**不能是 grounded**
- joint 的 **parent 要放 Component2**（`occurrenceTwo` 是 parent、`occurrenceOne` 是 child）
- 只支援 **Rigid / Revolute / Slider**，而且**不吃 As-built Joint**（只讀 `root.joints`）
- Revolute 的兩個 limit 要嘛都開、要嘛都不開。只開一邊會中止匯出；
  都不開會匯出成 `continuous`（輪子要的就是這個）

## 輸出

```
<save_dir>/
  urdf/        <robot>.xacro + materials / trans / gazebo
  meshes/      每個 link 一個 STL
  launch/      display / gazebo / controller
  <robot>_urdf/
    <robot>.urdf                        ← xacro include 全部展開的單一檔案
    <robot>_description/meshes/*.stl    ← 自包含的副本
```

`<robot>_urdf/` 那份給不吃 xacro 的工具用：Isaac Sim、PyBullet、MuJoCo、Drake、
Unity URDF-Importer 等等。

## 與原始專案的差異

### 1. limit 以匯出姿態為基準

Fusion 的 joint limit 是相對「建立 joint 當下」的姿態，而 URDF 的零位是
「匯出當下」的姿態。兩者不同時，直接搬運 Fusion 的數值會讓 limit 整個偏移。

`core/Joint.py` 改成扣掉當下的關節角度：

```python
rot_offset = joint.jointMotion.rotationValue
joint_dict['upper_limit'] = round(rot_limits.maximumValue - rot_offset, 6)
joint_dict['lower_limit'] = round(rot_limits.minimumValue - rot_offset, 6)
```

revolute 與 prismatic 各一處。

> 實例：零位角 45°、想要 ±110° 的範圍時，Fusion 顯示的絕對值是 −65°~+155°。
> 不扣掉偏移就會原封不動寫進 URDF，實際差了 45°。

### 2. 輸出不綁定特定模擬器

展開後的單一 URDF 原本叫 `<robot>_unity_urdf/`、模組叫 `xacro2unity.py`。
那份輸出對任何不吃 xacro 的工具都適用，所以改成 `<robot>_urdf/` 與 `xacro2urdf.py`。

## 已知限制

`core/Joint.py` 的 `make_joint_xml()` 把每個 joint 的 `effort` 和 `velocity`
寫死成 `100`：

```python
limit.attrib = {'upper': ..., 'lower': ..., 'effort': '100', 'velocity': '100'}
```

沒有在這裡改，是因為合理值依致動器而異。本專案用匯出後處理：
`step2urdf/urdf_set_effort/`。

## 授權與出處

MIT License，原作者 Toshinori Kitamura（見 `LICENSE`）。

衍生自 https://github.com/alianlbj23/fusion2urdf ，
該專案又 fork 自 https://github.com/syuntoku14/fusion2urdf 。

`utils/xacro2urdf.py` 參考 https://github.com/doctorsrn/xacro2urdf 。
