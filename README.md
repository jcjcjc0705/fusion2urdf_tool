# fusion2urdf_tool

把 Autodesk Fusion 的組裝設計轉成 URDF，再匯入 Isaac Sim 的一組工具。

從 STEP 或 IGES 匯入的設計通常不能直接匯出：零件的階層很深、有縫不起來的曲面、沒有質量、沒有 joint。這個 repo 把「整理設計到匯出」的每一步各做成一支 Fusion 腳本，另外附三個命令列工具，用來修改匯出後的 URDF。

匯出的是標準 URDF，不限 Isaac Sim，其他讀 URDF 的工具也能用。

## 工具一覽

Fusion 腳本，在 `fusion_scripts/`：

| 腳本 | 用途 | 設定方式 |
|---|---|---|
| CleanSurfaceBodies | 找出、隱藏或刪除沒有體積的曲面 body | `.py` 開頭的 `MODE` |
| ReplaceWithBox | 把選取的幾何換成同樣大小的實心方塊 | 對話框 |
| GroupBodiesToLink | 把選取的 body 收成一個 link | 對話框 |
| LinkReport | 唯讀。列出每個 link 的 body 數、質量、質心，找出沒被收進任何 link 的 body | `.py` 開頭的 `SOURCE_NAME` |
| SetMassMaterial | 指定材質和密度，或指定總質量 | 對話框 |
| JointSetup | 批次設定關節的零位和上下限，輸出起始姿態腳本 | `.py` 開頭的開關和 `JOINTS` 表 |
| URDF_Exporter | 匯出 URDF、xacro、STL | 對話框 |

命令列工具，在 repo 根目錄：

| 工具 | 用途 | 存到哪裡 |
|---|---|---|
| urdf_joint_reverse | 反轉關節的方向 | 直接改原檔 |
| urdf_set_effort | 修改關節的 effort 和 velocity 上限 | 直接改原檔 |
| urdf_zero_calibrator | 平移關節的零位 | 另存新檔 |

## 流程

1. [準備設計](#1-準備設計)：調整方向、關掉設計歷史
2. [清理曲面 body](#2-清理曲面-body)
3. [把 body 分成 link](#3-把-body-分成-link)
4. [設定質量](#4-設定質量)
5. [建立 joint](#5-建立-joint)
6. [設定零位和上下限](#6-設定零位和上下限)
7. [匯出 URDF](#7-匯出-urdf)
8. [修改 URDF](#8-修改-urdf)
9. [匯入 Isaac Sim](#9-匯入-isaac-sim)

## 需求

- Autodesk Fusion
- Python 3.9 以上，命令列工具使用，不需要安裝其他套件
- Isaac Sim，只有第 9 步需要。本文件以 5.1 的介面為例

## 在 Fusion 執行腳本

1. 上方工具列 UTILITIES → ADD-INS → Scripts and Add-Ins。
2. 按 `+` → Script or add-in from device。
3. 選 `fusion_scripts/` 底下那支腳本的資料夾。要選整個資料夾，裡面有一個 `.py` 和一個 `.manifest`。
4. 腳本出現在清單後，按執行。

有些腳本的設定寫在 `.py` 的開頭。用編輯器改好、存檔，再回 Fusion 執行。

## 1. 準備設計

### 調整方向

匯出器直接使用 Fusion 的世界座標，不會轉換座標軸。URDF 的慣例是 Z 軸朝上、X 軸朝前。模型在 Fusion 裡不是這個方向的話，先把整個模型轉過去：

1. 確認沒有 component 被固定（grounded，瀏覽器裡名稱旁會多一個固定圖示）。有的話對它按右鍵 → Unground from Parent。
2. 點第一個 component，按住 Shift 再點最後一個，選取全部。不要選到 Origin。
3. 按 `M`，Move Type 選 Rotate，Axis 選 Origin 裡要當旋轉軸的那一條，Angle 輸入角度，按 OK。

### 關掉設計歷史

對最外層的 root 按右鍵 → Change to Direct Modeling → Delete Parametric History。

後面的步驟會大量複製和刪除 body，關掉歷史後這些操作不會累積在 timeline 上。這個動作會永久刪除參數化歷史，請在設計的副本上做。

## 2. 清理曲面 body

曲面 body 沒有體積，算不出質量和慣量。匯出成 STL 時它卻會變成沒有厚度的薄殼，當成碰撞形狀容易穿透和抖動。匯入的設計裡常有這種縫不起來的曲面，匯出前要清掉。

### CleanSurfaceBodies

`CleanSurfaceBodies.py` 開頭的 `MODE` 決定這次執行做什麼：

| MODE | 動作 |
|---|---|
| `'report'` | 只報告：曲面 body 有幾個、分布在哪些 component |
| `'hide'` | 隱藏所有曲面 body |
| `'show'` | 顯示所有曲面 body，用來還原 `hide` |
| `'delete'` | 刪除所有曲面 body |

建議的順序：

1. `MODE = 'report'`，看有哪些曲面。
2. `MODE = 'hide'`，轉動模型，看隱藏後缺了什麼。
3. 缺的部分如果要保留，用 ReplaceWithBox 換成方塊。常見的例子是廠商提供的感測器模型，整個都是曲面。
4. `MODE = 'delete'`，刪掉剩下的曲面。

### ReplaceWithBox

把選取的幾何換成一個同樣大小的實心方塊，也就是它的軸對齊外接框。方塊是實體，有體積和慣量，匯出的 mesh 也很簡單。

執行後在對話框裡：

1. 選取要換掉的 component 或 body。
2. 「刪掉原本的 body」保持勾選。
3. 按「建立方塊」。

注意：

- 選 component 時，它和子 component 裡的所有 body 都會算進去，實體也包含在內。每個 component 各換成一個方塊。
- 方塊建在 component 的定義裡，所以同一個 component 的每個副本都會一起換掉。
- 「目標質量」的單位是公克。填了之後，結果視窗會算出方塊需要的密度，但不會套用。質量留到第 4 步用 SetMassMaterial 設定。

## 3. 把 body 分成 link

link 是 URDF 裡的一個剛體。匯出器的規則是：

- 每個 link 是 root 底下第一層的一個 component。
- link 裡面只有 body，沒有子 component。
- 機器人的基座必須命名為 `base_link`。

匯入的設計通常是很深的階層，所以要重新分組。

### GroupBodiesToLink

每執行一次建立一個 link：在 root 底下新增一個 component，把選取的 body 複製進去，位置不變。原本的 component 階層保持原樣，以下稱為來源樹。

1. 選取屬於同一個 link 的 body 或 component。選 component 會把它底下所有階層的 body 都收進去。
2. 輸入 Link 名稱。只能用英文字母、數字和底線，其他字元會自動換成底線。不能跟現有的 component 同名。
3. 按「建立 Link」。
4. 「建立後繼續建下一個」勾選時，對話框會自動重開，可以一路做完所有 link。

對話框裡另外有兩個選項，預設都勾選：

| 選項 | 勾選時 |
|---|---|
| 複製後隱藏來源 body | 收進 link 的來源 body 會被隱藏 |
| 忽略已隱藏的 body | 選取範圍裡已經隱藏的 body 不會被收進去 |

兩個都勾選時，來源樹裡還沒隱藏的 body 就是還沒分配的；把新建的 link 暫時隱藏，畫面上剩下的就是它們。最後可以直接選整棵來源樹，把剩下的 body 一次收成一個 link，通常就是 `base_link`。

### 檢查，然後刪掉來源樹

所有 link 建完後執行 LinkReport。它是唯讀的，不會修改設計。

執行前先把 `LinkReport.py` 開頭的 `SOURCE_NAME` 改成來源樹最上層 component 的名稱，預設是 `All`。它假設原始設計全部放在 root 底下的同一個 component 裡。

它會顯示兩個視窗：

1. component 清單：root 底下每個 component 的 body 數、質量、質心，以及整機的質量和質心。body 數分成「自持」和「含子階層」兩個數字。
2. 未認領比對：來源樹裡沒有被任何 link 收走的 body。

要確認的事：

- 每個 link 的「自持」和「含子階層」相等。不相等代表有 body 在子 component 裡，匯出時會漏掉。
- 有一個 link 叫 `base_link`。
- 未認領清單是空的，或只剩本來就不要的 body。

確認後刪掉來源樹。來源樹留著會跟 link 重疊，後面設質量、建 joint 時很容易選到來源樹的 body，而那些 body 不會成為 URDF 的一部分。

## 4. 設定質量

URDF 裡的慣量是匯出器用「幾何 × 密度」算出來的。只在 URDF 裡改 `<mass>`，`<inertia>` 還是錯的，所以質量要在 Fusion 裡設對。

執行 SetMassMaterial，在對話框裡：

1. 選取要設定的 body 或 component，可以多選。
2. 選模式、材質庫和材質：

   | 模式 | 要填的值 | 結果 |
   |---|---|---|
   | 設材質 | 密度（g/cm³），預設帶入材質原本的密度 | 套用材質。有改密度就以填的值為準 |
   | 設重量 | 目標總質量（g） | 自動算出密度，讓選取的 body 加起來等於目標質量 |

3. 按「套用」。結果視窗會顯示實際的密度和質量。

每次執行都會在設計裡複製一份新材質來用，不會改到材質庫，也不會影響其他 body。

「設重量」適合已經知道實際重量的零件，例如馬達、感測器，以及第 2 步換成的方塊。

全部設完後再執行一次 LinkReport，核對每個 link 和整機的質量。

## 5. 建立 joint

1. 對 `base_link` 按右鍵 → Ground to Parent。先把基座固定住，之後轉動關節時它才不會跟著動。
2. 按 `J` 新增 joint。
3. Component 1 選子 link，Component 2 選父 link。順序相反的話，URDF 裡的父子關係會顛倒。
4. Motion 的 Type 選 Revolute（旋轉）、Slider（平移）或 Rigid（固定）。
5. 按 Preview Motion 確認運動符合預期，再按確定。
6. 把 joint 改成有意義的名稱。這個名稱會直接成為 URDF 的 joint 名稱，下一步的 JointSetup 也用它對照。

規則：

- 用一般的 Joint，不要用 As-built Joint，匯出器讀不到。
- joint 要建在 root 這一層，而且兩端都要是 link 的 component，不能接到 root 或 Joint Origin。
- `base_link` 以外的每個 link，都要剛好是一個 joint 的 Component 1。沒有 joint 連到的 link 不會出現在 URDF 裡。
- 上下限的開關會決定匯出的類型：

  | Fusion 的類型 | 上下限都開 | 都不開 | 只開一個 |
  |---|---|---|---|
  | Revolute | `revolute` | `continuous`，輪子用這種 | 匯出中止 |
  | Slider | `prismatic` | 匯出的上下限都是 0，關節動不了 | 匯出中止 |
  | Rigid | `fixed` | `fixed` | `fixed` |

Revolute 的上下限可以留到下一步，用 JointSetup 一次設定。

## 6. 設定零位和上下限

### 零位是怎麼決定的

- Fusion 的 joint 角度 0，是建立 joint 當下的姿態，事後不能改。
- URDF 的角度 0，是匯出當下的姿態。
- 匯出器寫進 URDF 的上下限是「Fusion 的上下限 − 匯出當下的角度」。

所以做法是：先決定 URDF 的 0 度對應 Fusion 的幾度，這個角度稱為零位角；上下限以零位角為基準來設；匯出前把每個關節都轉到零位角。

JointSetup 把這些事做成批次。它只處理 Revolute joint，Slider 的上下限請在 Fusion 裡手動設定。

### JOINTS 表

`JointSetup.py` 開頭的 `JOINTS`，每一列是一個 joint。

腳本裡附的表是一台實際機器人的設定，當作範例：它有四條末端是輪子的腿，和兩隻末端是夾爪的手臂。使用時請整個換成你的 joint。下面節錄其中四列：

```python
JOINTS = [
    # 名稱             零位角   limit (下, 上)   反轉軸向   驅動到
    ('base_leg_rf',       0,    (-45,   45),    False,      0),   # 髖關節，接在 base_link 上
    ('leg_rf1_rf2',      45,    (-110, 110),    False,     45),   # 零位角不是 0
    ('leg_lr1_lr2',     -45,    (-110, 110),    True,     -45),   # 旋轉軸跟其他三條腿相反
    ('leg_rf_wheel',      0,    None,           False,      0),   # 輪子，不設限
]
```

| 欄位 | 意思 |
|---|---|
| 名稱 | Fusion 裡的 joint 名稱 |
| 零位角 | URDF 的 0 度對應 Fusion 的幾度 |
| limit | 上下限，以零位角為基準，也就是 URDF 會看到的範圍。`None` 代表不設限 |
| 反轉軸向 | `True` 代表這個關節的正方向要反過來 |
| 驅動到 | `DO_POSE` 要把關節轉到 Fusion 的幾度 |

角度的單位都是度。

Fusion 介面顯示的上下限是「零位角 + limit」，看起來不對稱是正常的。以上表的 `leg_rf1_rf2` 為例：零位角 45、limit −110 到 110，Fusion 顯示 −65 到 155，URDF 寫入 −110 到 110。

Fusion 不允許在 joint 建立後反轉軸向，所以「反轉軸向」只是記號。JointSetup 會列出這些 joint，匯出後再用 `urdf_joint_reverse` 處理，見第 8 步。

### 開關

| 開關 | 作用 |
|---|---|
| `DRY_RUN` | 不修改任何東西，只顯示對照表 |
| `DO_FREE` | 解除所有上下限，方便手動轉動關節。開了它，其他階段都不執行 |
| `DO_POSE` | 把每個關節轉到「驅動到」的角度。它會先解除所有上下限 |
| `DO_REVERSE` | 列出標記反轉的 joint，以及 URDF 裡的 `<axis>` 要改成什麼 |
| `DO_LIMITS` | 依表設定上下限 |
| `DO_DUMP_POSE` | 把目前的姿態輸出成 Isaac Sim 用的姿態腳本 |

同時開多個時，執行順序固定是 REVERSE → POSE → DUMP_POSE → LIMITS。

每次執行結束都會顯示對照表：每個 joint 的現在角、零位角、上下限，以及表和設計對不上的 joint。

不確定零位角是多少時：把關節轉到想當作 0 度的姿態，設 `DRY_RUN = True` 執行，對照表裡的「現在角」就是要填的零位角。

### 輸出起始姿態腳本

需要讓機器人在模擬裡以特定姿態開始時才做。

1. 表裡的名稱、零位角、反轉軸向填好最終的值。「驅動到」填起始姿態在 Fusion 裡的角度。
2. 只開 `DO_POSE` 和 `DO_DUMP_POSE`，執行，選擇存檔位置。

輸出的角度已經換算成 URDF 的角度：現在角減掉零位角，標記反轉的再變號。所以零位角和反轉軸向在這一步就必須是對的。

腳本裡第二張被註解掉的表，是範例機器人輸出站立姿勢時用的設定，可以對照著看：跟第一張只差在「驅動到」。

### 匯出前

1. 「驅動到」改成跟零位角一樣。
2. 開 `DO_POSE`、`DO_REVERSE`、`DO_LIMITS`，執行。
3. 看結果視窗：
   - 不能出現「現在角與零位角不符」的警告。有的話，匯出的零位和上下限都會偏。
   - 記下「匯出後要在 URDF 改的行」列出的 joint，第 8 步要用。

標記反轉的 joint 如果上下限不對稱，要注意表裡填的是 Fusion 方向的範圍。`urdf_joint_reverse` 反轉時會把上下限對調並變號。

## 7. 匯出 URDF

匯出前確認：

- 沒有 component 被固定。第 5 步固定的 `base_link` 要 Unground from Parent。
- 每個 link 都在 root 底下第一層，基座叫 `base_link`。
- 所有關節停在零位角，也就是剛做完第 6 步的「匯出前」。

執行 URDF_Exporter：

1. 選輸出資料夾。
2. 詢問要不要自動清理複製的 component 時，選 Yes。匯出 STL 時會暫時複製每個 link，名稱以 `exported_` 開頭，選 Yes 會在結束時刪掉它們。

輸出放在所選資料夾底下的 `<robot>_description/`。`<robot>` 是 Fusion 文件名稱的第一個單字。

```
<robot>_description/
├── urdf/              <robot>.xacro，以及 materials、trans、gazebo
├── meshes/            每個 link 一個 STL
├── launch/            display、gazebo、controller
├── CMakeLists.txt
├── package.xml
└── <robot>_urdf/
    ├── <robot>.urdf                     xacro 全部展開後的單一檔案
    └── <robot>_description/meshes/      STL 的副本
```

- 給 Isaac Sim 和其他不讀 xacro 的工具用的，是 `<robot>_urdf/<robot>.urdf`。這個資料夾自帶 mesh，可以整個搬到別處。
- 其餘的檔案是 ROS 1（catkin）套件的骨架。

匯出結果的特性：

- link 的名稱是 component 名稱加上副本編號，例如 component `arm` 匯出成 `arm_1`。基座固定叫 `base_link`。
- 每個 link 的 visual 和 collision 共用同一個 STL。
- 每個 joint 的 effort 和 velocity 都寫成 100，需要在第 8 步改成實際值。
- 長度單位是公尺。STL 以公釐儲存，URDF 裡用 `scale="0.001 0.001 0.001"` 換算。

## 8. 修改 URDF

三個工具都是互動式的：執行後列出所有 joint，照提示選擇和輸入。請對 `<robot>_urdf/<robot>.urdf` 執行。

```bash
python urdf_joint_reverse/main.py   path/to/<robot>.urdf
python urdf_set_effort/main.py      path/to/<robot>.urdf
python urdf_zero_calibrator/main.py path/to/<robot>.urdf
```

| 工具 | 什麼時候用 | 改了什麼 | 存到哪裡 |
|---|---|---|---|
| urdf_joint_reverse | 有 joint 在 JointSetup 標記了反轉 | `<axis>` 變號；上下限對調並變號；mimic 的 multiplier 變號 | 每改一個就存回原檔 |
| urdf_set_effort | 要把 effort 和 velocity 從 100 改成實際值 | `<limit>` 的 effort 和 velocity。continuous joint 沒有 `<limit>` 時會補一個 | 每改一次就存回原檔 |
| urdf_zero_calibrator | 匯出後才發現零位不對 | 把角度偏移寫進 `<origin rpy>`，可以選擇一併平移上下限 | 另存新檔，預設是 `<原檔名>_zero_calibrated.urdf` |

補充：

- urdf_set_effort 選 joint 時可以輸入編號、名稱、`1,3,5`、`7-12` 或 `all`。revolute 和 continuous 的單位是 N·m 和 rad/s，prismatic 是 N 和 m/s。它不會改動檔案的排版，也可以用在 `.xacro`。
- urdf_joint_reverse 會重寫整個檔案，排版會被重新整理。
- urdf_zero_calibrator 只支援 revolute 和 continuous。偏移的正方向以檔案裡當下的 `<axis>` 為準。照第 6 步在零位角匯出的話，用不到這個工具。
- 重新匯出會覆蓋 `<robot>.urdf`，這一步要重做。這些工具也不會同步修改 `urdf/` 裡的 xacro。

## 9. 匯入 Isaac Sim

以下以 Isaac Sim 5.1 為例。

1. File → New From Stage Template → Default Stage。
2. File → Import，選 `<robot>_urdf/<robot>.urdf`。
3. 設定匯入選項，按 Import。
4. 有起始姿態腳本的話：在模擬停止的狀態下，開 Window → Script Editor，貼上腳本內容並執行。它會設定每個關節的起始角度和 drive 的目標角度。

匯入選項：

| 區塊 | 選項 | 說明 |
|---|---|---|
| Model | Create in Stage / Referenced Model | Create in Stage 把機器人直接建在目前的場景裡。Referenced Model 會另外產生一個 USD 檔，預設放在 URDF 旁邊，場景用參照的方式載入它 |
| Model | Clear Stage on Import | 只在 Create in Stage 時出現。不勾選，才會保留目前場景的內容 |
| Links | Moveable Base / Static Base | 基座可以自由移動，或固定在世界上 |
| Links | Default Density | 只用在沒有質量的 link。匯出的 URDF 每個 link 都有質量，維持預設即可 |
| Joints & Drives | Joint Configuration | 增益的輸入方式。Stiffness 是直接填剛性和阻尼，Natural Frequency 是填自然頻率和阻尼比，再由匯入器換算 |
| Joints & Drives | Drive Type | Acceleration 會先除掉慣量，增益的效果不受質量影響。Force 以力或扭矩作用，效果會隨 link 的質量和慣量改變 |
| Joints & Drives | 每個 joint 的 Target | Position：位置控制，有剛性和阻尼。Velocity：速度控制，只有一個增益，適合輪子。None：不加控制，適合自己送扭矩 |
| Colliders | Collision From Visuals | 匯出的 URDF 每個 link 都已經有 collision，不用勾選 |
| Colliders | Collider Type | Convex Hull 把每個 link 包成一個凸包，計算快，但凹的地方會被填滿。Convex Decomposition 拆成多個凸塊，比較貼近外形 |
| Colliders | Allow Self-Collision | 是否計算機器人自己的 link 之間的碰撞 |
| Colliders | Replace Cylinders with Capsules | 只影響 URDF 裡的圓柱形狀。匯出的 URDF 只有 mesh，這個選項沒有作用 |

## 常見問題

| 現象 | 原因和處理 |
|---|---|
| 匯出時顯示 `There is no base_link` | 基座的 component 沒有命名為 `base_link` |
| 匯出時顯示 `... is missing lower limit` 或 `upper limit` | 那個 joint 只開了一邊的上下限。兩邊都開，或都關 |
| 匯出時顯示 `Invalid joint detected` | 那個 joint 沒有連接兩個 component，或是設計歷史殘留的 joint |
| 匯出失敗，錯誤訊息最後一行是 `KeyError` 加一個名稱 | 有 joint 接到不是 link 的 component，通常是沒刪掉的來源樹，或 link 裡的子 component |
| root 底下留著 `exported_` 開頭的 component | 匯出中途失敗，沒有清掉。設計歷史關閉時無法自動復原，手動刪掉再重新匯出 |
| URDF 的上下限或零位偏了 | 匯出時關節不在零位角。重做第 6 步的「匯出前」，確認沒有不符的警告 |
| 某個 link 的質量是 0，或少了零件 | 裡面是曲面 body，見第 2 步；或 body 在子 component 裡，LinkReport 的「自持」和「含子階層」會不相等 |
| 姿態腳本印出 `not found in stage` | 腳本裡的 joint 名稱跟場景裡的不一樣，或 URDF 還沒匯入 |

## 授權與出處

`fusion_scripts/URDF_Exporter` 以 MIT License 授權，原作者是 Toshinori Kitamura。它衍生自 [alianlbj23/fusion2urdf](https://github.com/alianlbj23/fusion2urdf)，後者 fork 自 [syuntoku14/fusion2urdf](https://github.com/syuntoku14/fusion2urdf)。`utils/xacro2urdf.py` 參考 [doctorsrn/xacro2urdf](https://github.com/doctorsrn/xacro2urdf)。跟原版的差異見 [fusion_scripts/URDF_Exporter/README.md](fusion_scripts/URDF_Exporter/README.md)。

個別工具的說明：

- [fusion_scripts/URDF_Exporter/README.md](fusion_scripts/URDF_Exporter/README.md)
- [urdf_joint_reverse/README.md](urdf_joint_reverse/README.md)
- [urdf_zero_calibrator/README.md](urdf_zero_calibrator/README.md)
