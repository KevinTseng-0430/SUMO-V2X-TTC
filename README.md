# SUMO V2X 車輛追撞預警情境

這個專案建立一個可重現的單車道緊急煞車情境，將前、後車狀態透過 UE
網路介面送至 MEC，並讓 MEC 警告的實際抵達時間影響後車煞車時機。

## 情境時間線

```text
0.0 s                  5.0 s                 warning + 0.3 s
  │                      │                         │
  ├── 兩車 20 m/s 巡航 ──┼── 前車以 8 m/s² 急煞 ──┼── 後車開始煞車
  │     淨車距 28 m       │                         │
  └──────────────────────┴─────────────────────────┴────────────
```

- `leader_car`：紅色前車。
- `follower_car`：藍色後車；收到警告時轉為黃色。
- 未收到 MEC 警告時，後車在 1.5 秒的駕駛反應時間後才煞車。
- 預設每 0.1 秒向 MEC 回報一次，也就是 10 Hz。
- 模擬預設和真實時間同步，HTTP 在背景執行，不會凍結 SUMO 時鐘。

GUI 場景包含測試區標線、草地、步道、控制中心、停車區、樹列、車名及
V2X/MEC 標示。GUI 會自動追蹤後車。

## 檔案

| 檔案 | 用途 |
|---|---|
| `sumo_ue_sender.py` | TraCI 情境控制、非同步 MEC 傳輸及量測 |
| `rear_end.sumocfg` | SUMO 主設定 |
| `traffic.rou.xml` | 車型、初始位置與路線 |
| `nodes.nod.xml`、`edges.edg.xml` | 可編輯路網來源 |
| `road.net.xml` | `netconvert` 產生的 SUMO 路網 |
| `visuals.add.xml` | 道路周邊場景與標示 |
| `viewsettings.xml` | GUI 色彩、標籤、縮放及顯示樣式 |
| `mock_mec_server.py` | 本機測試用 MEC 相容伺服器 |
| `outputs/` | 每次執行的 CSV、JSON、FCD、tripinfo 與碰撞輸出 |

## 快速開始

### 1. 不連 MEC 的基準測試

快速 headless 驗證：

```bash
python3 sumo_ue_sender.py --dry-run
```

以 GUI 及真實時間觀看基準情境：

```bash
python3 sumo_ue_sender.py --mode baseline --gui --realtime
```

這個設定預期會發生追撞，用來作為 MEC 預警的對照組。

### 2. 使用本機 mock MEC

第一個終端機：

```bash
python3 mock_mec_server.py
```

第二個終端機：

```bash
python3 sumo_ue_sender.py \
  --mode mec \
  --mec-url http://127.0.0.1:18080/mec/v2x/sumo/report \
  --ue-interface '' \
  --gui \
  --realtime
```

mock MEC 預設於 TTC 小於等於 4 秒時回傳警告，並加入 10 ms 處理延遲。
可使用 `--delay-ms` 模擬更高的伺服器延遲。

### 3. 使用真實 UE 與 MEC

先確認 UERANSIM UE tunnel 已存在：

```bash
ip link show uesimtun0
```

再執行：

```bash
python3 sumo_ue_sender.py \
  --mode mec \
  --mec-url http://172.16.6.100/mec/v2x/sumo/report \
  --ue-interface uesimtun0 \
  --gui \
  --realtime
```

URL、介面及 UE ID 也可由環境變數設定：

```bash
export MEC_URL=http://172.16.6.100/mec/v2x/sumo/report
export UE_INTERFACE=uesimtun0
export UE_ID=ueransim-ue-001
python3 sumo_ue_sender.py
```

若不指定 tunnel，可傳入 `--ue-interface ''`，此時 curl 使用作業系統的一般
路由表。

## 執行模式

| 模式 | 行為 |
|---|---|
| `mec` | 傳送車況並使用第一個有效、未過期的 MEC 警告 |
| `baseline` | 不連 MEC，只使用 1.5 秒駕駛反應時間 |
| `mec-timeout` | 不產生網路請求，重現 MEC 無回應時的 fallback |

常用參數：

```text
--send-rate 10
--warning-reaction-time 0.3
--fallback-reaction-time 1.5
--max-response-age 1.0
--mec-timeout 2.0
--max-inflight 4
--no-gui
--no-realtime
```

`--no-realtime` 適合 baseline 與離線測試。正式 MEC 實驗應保留
`--realtime`，否則模擬可能比 HTTP 回應更快跑完。

## MEC JSON 契約

既有必要請求欄位保持不變：

```json
{
  "scenario": "rear_end_emergency_brake",
  "ue_id": "ueransim-ue-001",
  "sim_time": 5.8,
  "client_send_ts": 1785139194.8,
  "vehicles": [
    {
      "vehicle_id": "leader_car",
      "x": 198.0,
      "y": -1.6,
      "speed": 13.6,
      "accel": -8.0,
      "lane_id": "main_road_0",
      "lane_pos": 198.0,
      "length": 5.0
    }
  ]
}
```

另外加入 `run_id`、`sequence` 與 `event_state`，舊版 MEC 可直接忽略。

預期回應中的風險結果：

```json
{
  "server_recv_ts": 1785139194.81,
  "server_send_ts": 1785139194.82,
  "proc_delay_ms": 10.0,
  "client_ip": "10.0.0.2",
  "result": {
    "gap": 24.4,
    "relative_speed": 7.2,
    "ttc": 3.39,
    "risk_level": "medium",
    "warning": true,
    "leader_id": "leader_car",
    "follower_id": "follower_car"
  }
}
```

只有 `warning` 嚴格等於 JSON boolean `true`、車輛 ID 相符，且回應未超過
`--max-response-age` 時，警告才會被採用。

## 輸出

每次執行使用時間與隨機碼產生獨立 `run_id`，或以 `--run-id` 指定。

- `<run_id>_steps.csv`：每個 SUMO step 的本地 ground truth。
- `<run_id>_network.csv`：每個 MEC 請求的 RTT、伺服器時間與風險結果。
- `<run_id>_summary.json`：碰撞結果、最小 gap/TTC、警告與煞車時間。
- `<run_id>_fcd.xml`：SUMO 浮動車輛資料。
- `<run_id>_tripinfo.xml`：旅程資料。
- `<run_id>_collisions.xml`：SUMO 碰撞紀錄。

## 修改路網

更新 `nodes.nod.xml` 或 `edges.edg.xml` 後重新產生路網：

```bash
netconvert \
  --node-files nodes.nod.xml \
  --edge-files edges.edg.xml \
  --output-file road.net.xml \
  --no-turnarounds true
```

目前已在 Eclipse SUMO 1.27.0 驗證。Python 會先嘗試一般 `import traci`，
失敗時自動載入 `/usr/share/sumo/tools` 或 `$SUMO_HOME/tools`。
