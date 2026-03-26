# GUI Design Guide

Design rules for PyQt5 GUI tools in this project. All GUIs follow the **Catppuccin Mocha** dark theme for visual consistency.

Reference implementation: `src/tools/ethercat_tools/ethercat_tools/demo_ethercat_status/`

---

## Color Palette (Catppuccin Mocha)

### Backgrounds
| Role | Hex | Usage |
|------|-----|-------|
| Base | `#1e1e2e` | Window background, table background, text edit background |
| Surface 0 | `#313244` | Cards, frames, status bars, table headers, inactive tabs |
| Surface 1 | `#45475a` | Borders, grid lines, active tabs, dividers |

### Text
| Role | Hex | Usage |
|------|-----|-------|
| Text | `#cdd6f4` | Primary text, labels, table content |
| Subtext 0 | `#6c7086` | Secondary labels, field captions, dimmed text |

### Semantic Colors
| Role | Hex | Name | Usage |
|------|-----|------|-------|
| OK / Good | `#a6e3a1` | Green | Health OK, operational states, clear status, log output |
| Warning | `#f9e2af` | Yellow | Warnings, transitional states (Ready, Switched On), demo mode |
| Error / Fault | `#f38ba8` | Red | Faults, errors, e-stop, disconnected states |
| Info / Accent | `#89b4fa` | Blue | Raw data, hex displays, informational highlights |
| Notice | `#fab387` | Peach | Degraded states (Switch On Disabled), secondary warnings |

### Usage Rules
- Never use raw white (`#ffffff`) or black (`#000000`)
- All status indicators must use the semantic palette above — do not invent new colors
- Use `Subtext 0` for field caption labels (the small text above values)
- Use `Text` for the values themselves

---

## Typography

| Context | Font | Size | Weight |
|---------|------|------|--------|
| Status bar headline | System | 14px | Bold |
| Field value | System | 13px | Bold |
| Field caption | System | 10px | Normal |
| Table content | Monospace | 12px | Normal |
| Table header | System | 12px | Bold |
| Log / raw data | Monospace | 11px | Normal |

### Rules
- Numeric data (positions, velocities, hex values, timestamps) always uses `font-family: monospace`
- Captions use the smaller 10px size in `#6c7086` — they go above the value, not beside it
- Format numbers with fixed-width format specifiers (`f"{val:+8.2f}"`) for column alignment

---

## Layout Structure

### Standard 3-Section Layout

Every monitoring/status GUI follows this vertical layout:

```
┌─────────────────────────────────────────────┐
│  Status Bar (QFrame, 60-70px height)        │  ← Fixed height
│  [State] [Metric] [Metric] [Metric] ...    │
├─────────────────────────────────────────────┤
│                                             │
│  Main Data Table (QTableWidget)             │  ← Stretches
│                                             │
├─────────────────────────────────────────────┤
│  Diagnostics Tabs (QTabWidget)              │  ← Stretches
│  [Tab 1] [Tab 2] [Tab 3]                   │
│                                             │
└─────────────────────────────────────────────┘
```

### Spacing
| Property | Value |
|----------|-------|
| Main layout margins | 12px all sides |
| Main layout spacing | 8px between sections |
| Status bar internal spacing | 24px between metrics |
| Status bar left/right padding | 16px |
| Tab content padding | 8px |

---

## Status Bar Pattern

The top status bar uses a `QFrame` with `#313244` background and `border-radius: 8px`. Internal layout is `QHBoxLayout`.

Each metric is a vertical pair: caption label on top, value label below.

```
┌──────────────────────────────────────────────────────────────┐
│ OPERATIONAL  │ Interface │ Slaves │ Cycle Rate │ Uptime │ ...│
│              │ enp3s0    │ 6 / 6  │ 1000 Hz    │ 00:05:23   │
└──────────────────────────────────────────────────────────────┘
```

The first element is the primary state indicator (large bold text, color-coded). Remaining items are caption+value pairs. End with a horizontal spacer.

```python
# Caption label style
"color: #6c7086; font-size: 10px;"

# Value label style
"font-size: 13px; font-weight: bold;"

# State indicator style (color varies by state)
"color: #a6e3a1; font-size: 14px; font-weight: bold;"
```

---

## Table Patterns

### QTableWidget Config
```xml
<property name="alternatingRowColors"><bool>true</bool></property>
<property name="selectionBehavior"><enum>QAbstractItemView::SelectRows</enum></property>
```

### In Python Setup
```python
table.horizontalHeader().setStretchLastSection(True)
table.verticalHeader().setVisible(False)
table.resizeColumnsToContents()
```

### Cell Helper
All table cell updates go through a shared helper to ensure consistent formatting:

```python
@staticmethod
def _set_cell(table, row, col, text, color=None):
    item = table.item(row, col)
    if item is None:
        item = QTableWidgetItem()
        item.setFlags(item.flags() & ~Qt.ItemIsEditable)
        table.setItem(row, col, item)
    item.setText(str(text))
    if color:
        item.setForeground(QColor(color))
    item.setTextAlignment(Qt.AlignCenter)
```

### Health / Status Columns
Always use color-coded text for status cells:

| State | Text | Color |
|-------|------|-------|
| Nominal | `OK` | `#a6e3a1` |
| Warning | `WARN` | `#f9e2af` |
| Fault | `FAULT` | `#f38ba8` |

---

## Diagnostic Tabs

Use `QTabWidget` for the bottom section. Standard tabs to include where applicable:

1. **Fault Log** — `QTextEdit`, read-only, monospace 11px, green text (`#a6e3a1`). Timestamped entries.
2. **Safety Status** — Summary indicators (e-stop, watchdog) + per-joint limit table.
3. **Raw Data** — `QTextEdit`, read-only, monospace 11px, blue text (`#89b4fa`). Hex or protocol-level data.

---

## File Organization

```
src/tools/<tool_name>/
├── CMakeLists.txt
├── package.xml
├── setup.cfg
├── resource/<tool_name>           # Empty ament index marker
├── scripts/<executable_name>      # Entry point wrapper
├── <tool_name>/
│   ├── __init__.py
│   └── <demo_or_feature>/
│       ├── __init__.py
│       ├── main.py                # ROS node + QApplication bootstrap
│       ├── <name>_window.py       # QMainWindow subclass
│       └── <name>.ui              # Qt Designer UI file
```

### Entry Point Script (`scripts/<name>`)
```python
#!/usr/bin/env python3
from <tool_name>.<module>.main import main
main()
```

### Main Bootstrap (`main.py`)
```python
def main():
    app = QApplication(sys.argv)
    ros_node = None

    try:
        import rclpy
        rclpy.init(args=sys.argv)
        ros_node = rclpy.create_node("<node_name>")
        window = MyWindow(ros_node=ros_node)
        # Subscribe to topics here
        ros_timer = QTimer()
        ros_timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
        ros_timer.start(10)
    except Exception:
        window = MyWindow(ros_node=None)  # Demo mode fallback

    window.show()
    ret = app.exec_()
    # Cleanup...
    sys.exit(ret)
```

### Window Class (`*_window.py`)
- Loads UI via `uic.loadUi(path, self)` — no compiled `.py` from `.ui`
- UI file located using local path first, `ament_index` fallback
- `QTimer` at 10 Hz (100ms) for display updates
- Demo mode generates simulated data when `ros_node is None`

---

## CMakeLists.txt Template

```cmake
cmake_minimum_required(VERSION 3.14)
project(<tool_name>)

find_package(ament_cmake REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  FILES <tool_name>/<module>/<name>.ui
  DESTINATION share/${PROJECT_NAME}/ui
)

install(
  PROGRAMS scripts/<executable_name>
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
```

---

## Global Stylesheet

Apply this on the `QMainWindow` in the `.ui` file so all child widgets inherit:

```css
QMainWindow { background-color: #1e1e2e; color: #cdd6f4; }
QLabel { color: #cdd6f4; }
QFrame { background-color: #313244; border-radius: 8px; }
QTableWidget { background-color: #1e1e2e; color: #cdd6f4; gridline-color: #45475a; border: none; font-family: monospace; font-size: 12px; }
QTableWidget::item { padding: 4px 8px; }
QTableWidget QHeaderView::section { background-color: #313244; color: #cdd6f4; border: 1px solid #45475a; padding: 6px; font-weight: bold; font-size: 12px; }
QTabWidget::pane { border: 1px solid #45475a; background-color: #1e1e2e; border-radius: 4px; }
QTabBar::tab { background-color: #313244; color: #cdd6f4; padding: 8px 16px; margin-right: 2px; border-top-left-radius: 4px; border-top-right-radius: 4px; }
QTabBar::tab:selected { background-color: #45475a; color: #cdd6f4; }
QTextEdit { background-color: #1e1e2e; color: #a6e3a1; border: none; font-family: monospace; font-size: 11px; }
QGroupBox { color: #cdd6f4; border: 1px solid #45475a; border-radius: 6px; margin-top: 8px; padding-top: 16px; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
```
