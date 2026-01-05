import sys 
import math
import datetime
from collections import deque
#imported all the pyqt pack for the graphickal
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QPushButton, QComboBox, 
                             QSpinBox, QGraphicsScene, QGraphicsView, 
                             QGroupBox, QFormLayout, QDoubleSpinBox, 
                             QDialog, QLineEdit, QDialogButtonBox, QGraphicsEllipseItem, 
                             QGraphicsLineItem, QTableWidget, QTableWidgetItem, QHeaderView, 
                             QListWidget, QCheckBox, QTabWidget, QTextEdit, QMenu, QAbstractItemView)
from PyQt5.QtCore import Qt, QLineF, QPointF
from PyQt5.QtGui import QPen, QBrush, QColor, QPainter, QFont, QPolygonF, QFontMetrics

# min heap class for algorithms

class Heap :
    def __init__(self):
        self.heap = list()
    
    def push (self,value):
        self.heap.append(value)
        self._up_(len(self.heap)-1)
    
    def pop(self):
        if not self.heap:
            raise IndexError("pop from empty heap")
        last = self.heap.pop()
        if not self.heap :
            return last
        _min = self.heap[0]
        self.heap[0] = last
        self._down_(0)
        return _min
    
    def _up_(self,index):
        parent = (index-1)>>1
        while index > 0 and self.heap[index] < self.heap[parent]:
            self.heap[index],self.heap[parent] = self.heap[parent],self.heap[index]
            index = parent
            parent = (index-1)>>1
    
    def _down_(self,index):
        length = len(self.heap)
        while True :
            left =(index << 1) + 1
            right = left + 1
            small = index
            if left < length and self.heap[left] < self.heap[small]:
                small = left
            if right < length and self.heap[right] < self.heap[small]:
                small = right 
            
            if small == index :
                break
            self.heap[index],self.heap[small] = self.heap[small],self.heap[index]
            index = small 
    
    def __len__(self):
        return len(self.heap)
    
# Logical Data Structures for algorithms

class Transport:
    def __init__(self):
        self.nodes = dict() 
        self.edges = dict()
    
    def add_node(self,id,x,y):
        self.nodes[id] = (x,y)
        if id not in self.edges:
            self.edges[id] = [] 
    
    def remove_node(self,id):
        if id in self.nodes:
            del self.nodes[id]
        if id in self.edges:
            del self.edges[id]
        
        for i in self.edges:
            self.edges[i] = [j for j in self.edges[i] if j['to'] != id]

# this part add and remove the edges

    def add_edge(self,u,v,duration,cost,transport_type,schedule=None):
        if u in self.nodes and v in self.nodes :
            self.edges[u].append({
                'to':v,
                'duration':duration,
                'cost':cost,
                'type':transport_type,
                'schedule':sorted(schedule) if schedule else []
            })
    
    def remove_edge(self, u, v, edge_data):
        if u in self.edges:
            self.edges[u] = [e for e in self.edges[u] if not (
                e['to'] == v and 
                e['type'] == edge_data['type'] and 
                e['base_time'] == edge_data['base_time']
            )]

    def get_heuristic(self, u, v):
        if u not in self.nodes or v not in self.nodes: return 0
        x1, y1 = self.nodes[u]
        x2, y2 = self.nodes[v]
        return math.sqrt((x1-x2)**2 + (y1-y2)**2) * 0.1 

    def clear(self):
        self.nodes = {}
        self.edges = {}



# algorithms is A star / Dijkstra / BFS / maybe DFS

class PathFinder:
    def __init(self,graph):
        self.graph = graph

    def get_wait(self,time_now,schedule,transport_type):
        if transport_type in ['walk','taxi']:
            return 0
        if not schedule:
            return 0
        time = int(1440-time_now) + schedule[0]

        for i in schedule:
            if i >= time_now:
                return i - time_now
        
        return (1440 - time) + schedule[0]
    def solve(self, start, end, start_time, budget, traffic_factor, allowed_modes, max_duration, algorithm='dijkstra', objective='fastest'):
        if start not in self.graph.nodes or end not in self.graph.nodes:
            return None 
            
        if algorithm == 'bfs':
            return self._bfs(start, end, start_time, budget, traffic_factor, allowed_modes)
        else:
            use_heuristic = (algorithm == 'astar')
            return self._dijkstra_astar(start, end, start_time, budget, traffic_factor, allowed_modes, max_duration, objective, use_heuristic)

    def _dijkstra_astar(self, start, end, start_time, budget, traffic_factor, allowed_modes, max_duration, objective, use_heuristic):
        push_count = 0
        pq = Heap()
        pq.push((0, start_time, 0, push_count, start, []))
        
        visited = {} 
        min_final_metric = float('inf')
        best_result = None

        while len(pq) > 0:
            score, curr_time, curr_cost, _, u, path_history = pq.pop()
            if max_duration > 0 and (curr_time - start_time) > max_duration:
                continue
            if objective == 'fastest':
                current_metric = curr_time
            elif objective == 'cheapest':
                current_metric = curr_cost
            else: 
                current_metric = sum(1 for step in path_history if step['type'] == 'walk') * 100 

            if u in visited and visited[u] <= current_metric:
                continue
            visited[u] = current_metric

            if u == end:
                if current_metric < min_final_metric:
                    min_final_metric = current_metric
                    best_result = (path_history, curr_time, curr_cost)
                continue

            if u not in self.graph.edges: continue

            for edge in self.graph.edges[u]:
                v = edge['to']
                e_type = edge['type']
                
                if e_type not in allowed_modes: continue
                speed_multiplier = 1.0
                cost_multiplier = 1.0
                if e_type == 'taxi':
                    speed_multiplier = traffic_factor 
                    cost_multiplier = traffic_factor  
                elif e_type == 'bus':
                    speed_multiplier = 1.0 + (traffic_factor - 1.0) * 0.3 

                travel_time = edge['base_time'] * speed_multiplier
                final_edge_cost = edge['cost'] * cost_multiplier

                if curr_cost + final_edge_cost > budget:
                    continue 

                wait_time = self.get_wait_time(curr_time, edge['schedule'], e_type)
                arrival_time = curr_time + wait_time + travel_time
                new_cost = curr_cost + final_edge_cost
                
                g_score = 0
                if objective == 'fastest': g_score = arrival_time
                elif objective == 'cheapest': g_score = new_cost
                elif objective == 'least_walking': 
                    walk_penalty = 1000 if e_type == 'walk' else 0
                    g_score = score + walk_penalty

                h_score = 0
                if use_heuristic and objective == 'fastest':
                    h_score = self.graph.get_heuristic(v, end)
                    
                f_score = g_score + h_score
                
                step_detail = {
                    'from': u, 'to': v, 'type': e_type,
                    'wait': wait_time, 'travel': travel_time,
                    'cost': final_edge_cost, 'arrival': arrival_time
                }

                push_count += 1
                pq.push((f_score, arrival_time, new_cost, push_count, v, path_history + [step_detail]))

        return best_result

def _bfs(self, start, end, start_time, budget, traffic_factor, allowed_modes):
        queue = deque([(start, [start])])
        visited_hops = {start}
        shortest_path_nodes = None

        while queue:
            u, path = queue.popleft()
            if u == end: 
                shortest_path_nodes = path
                break
            if u in self.graph.edges:
                for edge in self.graph.edges[u]:
                    v = edge['to']
                    if edge['type'] not in allowed_modes: continue
                    if v not in visited_hops:
                        visited_hops.add(v)
                        queue.append((v, path + [v]))
        
        if not shortest_path_nodes: return None

        path_history = []
        curr_time = start_time
        curr_cost = 0

        for i in range(len(shortest_path_nodes) - 1):
            u, v = shortest_path_nodes[i], shortest_path_nodes[i+1]
            
            best_edge = None
            best_metric = float('inf')

            for edge in self.graph.edges[u]:
                if edge['to'] == v and edge['type'] in allowed_modes:
                    speed_mult = traffic_factor if edge['type'] == 'taxi' else 1.0
                    cost_mult = traffic_factor if edge['type'] == 'taxi' else 1.0
                    
                    wait = self.get_wait_time(curr_time, edge['schedule'], edge['type'])
                    travel = edge['base_time'] * speed_mult
                    cost = edge['cost'] * cost_mult
                    
                    total_time = wait + travel
                    if total_time < best_metric: 
                        best_metric = total_time
                        best_edge = edge
                        best_vals = (wait, travel, cost)

            if not best_edge: return None
            wait, travel, cost = best_vals
            
            curr_time += (wait + travel)
            curr_cost += cost
            
            if curr_cost > budget: return None 
            
            path_history.append({
                'from': u, 'to': v, 'type': best_edge['type'],
                'wait': wait, 'travel': travel,
                'cost': cost, 'arrival': curr_time
            })

        return (path_history, curr_time, curr_cost)


# GUI 
class StationItem(QGraphicsEllipseItem):
    def __init__(self, x, y, id, main_window, radius=20):
        super().__init__(x - radius, y - radius, radius * 2, radius * 2)
        self.id = id
        self.main_window = main_window
        
        gradient = QColor("#0a2a4b")
        self.setBrush(QBrush(gradient)) 
        self.setPen(QPen(QColor("#ecf0f1"), 2))  
        self.setZValue(10)
        self.setAcceptHoverEvents(True)
        self.setToolTip(f"Station: {id}\n(Right-Click to delete)")

    def paint(self, painter, option, widget):
        super().paint(painter, option, widget)
        painter.setPen(Qt.white)
        font = QFont("Berlin sans FB Demi", 10, QFont.Bold)
        painter.setFont(font)
        rect = self.boundingRect()
        text = self.id
        fm = QFontMetrics(font)
        w = fm.width(text)
        painter.drawText(int(rect.center().x() - w/2), int(rect.center().y() + 4), text)
    def contextMenuEvent(self, event):
        menu = QMenu()
        del_action = menu.addAction(f"Delete Station {self.id}")
        action = menu.exec_(event.screenPos())
        if action == del_action:
            self.main_window.delete_node_action(self.id)

class RouteItem(QGraphicsLineItem):
    def __init__(self, u, v, u_pos, v_pos, edge_data, main_window):
        super().__init__(QLineF(u_pos[0], u_pos[1], v_pos[0], v_pos[1]))
        self.edge_data = edge_data
        self.u = u
        self.v = v
        self.main_window = main_window
        self.setAcceptHoverEvents(True)
        self.update_style()
    def update_style(self):
        color = QColor("#7f8c8d") 
        width = 2
        style = Qt.SolidLine
        etype = self.edge_data['type']
        
        if etype == 'metro': color = QColor("#3498db") 
        elif etype == 'bus': color = QColor("#e74c3c") 
        elif etype == 'taxi': color = QColor("#f1c40f") 
        elif etype == 'walk': 
            color = QColor("#2ecc71") 
            style = Qt.DashLine
        self.pen_normal = QPen(color, width, style)
        self.pen_hover = QPen(QColor("#ffffff"), 4, style) 
        self.setPen(self.pen_normal)
        sched_txt = "Anytime"
        if self.edge_data['schedule']:
            sched_txt = ",".join(map(str, self.edge_data['schedule'][:3])) + "..."
        tooltip = (f"Mode: {etype.upper()}\n"
                   f"Time: {self.edge_data['base_time']} min\n"
                   f"Cost: ${self.edge_data['cost']}\n"
                   f"Sched: {sched_txt}")
        self.setToolTip(tooltip)
    def hoverEnterEvent(self, event):
        self.setPen(self.pen_hover)
        super().hoverEnterEvent(event)

    def hoverLeaveEvent(self, event):
        self.setPen(self.pen_normal)
        super().hoverLeaveEvent(event)

    def contextMenuEvent(self, event):
        menu = QMenu()
        del_action = menu.addAction("Remove Route")
        action = menu.exec_(event.screenPos())
        if action == del_action:
            self.main_window.graph.remove_edge(self.u, self.v, self.edge_data)
            self.main_window.refresh_map()

# The State ment of edge for u to v example of this .
class AddEdgeDialog(QDialog):
    def __init__(self, u, v, parent=None):
        super().__init__(parent)
        self.setWindowTitle(f"Connect {u} -> {v}")
        self.resize(300, 200)
        layout = QFormLayout(self)

        self.cmb_type = QComboBox()
        self.cmb_type.addItems(["metro", "bus", "taxi", "walk"])
        self.cmb_type.currentTextChanged.connect(self.update_defaults)
        
        self.spin_duration = QSpinBox()
        self.spin_duration.setRange(1, 1000); self.spin_duration.setValue(10)
        self.spin_cost = QSpinBox()
        self.spin_cost.setRange(0, 100000); self.spin_cost.setValue(2)
        
        self.txt_schedule = QLineEdit()
        self.txt_schedule.setPlaceholderText("Mins (e.g. 0, 15, 30)")
        
        layout.addRow("Type:", self.cmb_type)
        layout.addRow("Duration (min):", self.spin_duration)
        layout.addRow("Cost ($):", self.spin_cost)
        layout.addRow("Schedule:", self.txt_schedule)

        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)
        self.update_defaults()

    def update_defaults(self):
        ctype = self.cmb_type.currentText()
        if ctype == "walk":
            self.spin_cost.setValue(0); self.spin_cost.setEnabled(False)
            self.txt_schedule.clear(); self.txt_schedule.setEnabled(False)
        elif ctype == "taxi":
            self.spin_cost.setValue(20); self.spin_cost.setEnabled(True)
            self.txt_schedule.clear(); self.txt_schedule.setEnabled(False)
        else:
            self.spin_cost.setEnabled(True); self.txt_schedule.setEnabled(True)

    def get_data(self):
        sched_str = self.txt_schedule.text().strip()
        ctype = self.cmb_type.currentText()
        schedule = []
        if ctype not in ['taxi', 'walk'] and sched_str:
            try: schedule = [int(x.strip()) for x in sched_str.split(',')]
            except: pass 
        return {'type': ctype, 'duration': self.spin_duration.value(), 'cost': self.spin_cost.value(), 'schedule': schedule}
    
#Map Loading
class MapScene(QGraphicsScene):
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
        self.mode = "view"
        self.grid_size = 50
        self.setBackgroundBrush(QBrush(QColor("#082841")))

    def drawBackground(self, painter, rect):
        super().drawBackground(painter, rect)
        painter.setPen(QPen(QColor("#1d57d4"), 1, Qt.DotLine))
        left = int(rect.left()) - (int(rect.left()) % self.grid_size)
        top = int(rect.top()) - (int(rect.top()) % self.grid_size)
        x = left
        while x < rect.right():
            painter.drawLine(x, int(rect.top()), x, int(rect.bottom()))
            x += self.grid_size
        y = top
        while y < rect.bottom():
            painter.drawLine(int(rect.left()), y, int(rect.right()), y)
            y += self.grid_size

    def mousePressEvent(self, event):
        if self.mode == "edit" and event.button() == Qt.LeftButton:
            pos = event.scenePos()
            if not self.itemAt(pos, self.views()[0].transform()):
                self.main_window.add_node_visual(pos.x(), pos.y())
        super().mousePressEvent(event)


# The main window done set.
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ProTransit - S1 System")
        self.resize(1200, 800)
        self.setStyleSheet("""
            QMainWindow, QWidget { background-color: #2f3640; color: #f5f6fa; font-family: 'Segoe UI', Arial; font-size: 14px; }
            QPushButton { background-color: #0984e3; border: none; padding: 8px; border-radius: 4px; color: white; }
            QPushButton:hover { background-color: #74b9ff; }
            QGroupBox { border: 1px solid #7f8c8d; border-radius: 5px; margin-top: 10px; padding-top: 10px; font-weight: bold; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; color: #00d2d3; }
            QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox { background-color: #353b48; border: 1px solid #718093; color: white; padding: 4px; border-radius: 3px; }
            QTableWidget { background-color: #353b48; border: none; gridline-color: #718093; }
            QHeaderView::section { background-color: #2f3640; padding: 4px; border: 1px solid #718093; }
            QTabBar::tab { background: #353b48; color: #bdc3c7; padding: 10px; margin-right: 2px; }
            QTabBar::tab:selected { background: #0984e3; color: white; }
        """)
        self.graph = Transport()
        self.finder = PathFinder(self.graph)
        self.group_travelers = [] 

        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        self.sidebar = QTabWidget()
        self.sidebar.setFixedWidth(420)
        
        self.tab_route = QWidget(); self.setup_route_tab()
        self.tab_group = QWidget(); self.setup_group_tab()
        self.tab_build = QWidget(); self.setup_build_tab()
        
        self.sidebar.addTab(self.tab_route, "Routing")
        self.sidebar.addTab(self.tab_group, "Group Mode")
        self.sidebar.addTab(self.tab_build, "Map Builder")
        # Graphics Scene
        self.scene = MapScene(self)
        self.map_view = QGraphicsView(self.scene)
        self.map_view.setRenderHint(QPainter.Antialiasing)
        self.map_view.setDragMode(QGraphicsView.ScrollHandDrag)

        main_layout.addWidget(self.sidebar)
        main_layout.addWidget(self.map_view, 1)

        self.load_demo_map()

    def format_time(self, mins):
        h = int(mins // 60) % 24
        m = int(mins % 60)
        return f"{h:02d}:{m:02d}"

    def setup_route_tab(self):
        layout = QVBoxLayout(self.tab_route)
        
        grp_trip = QGroupBox("Trip Details")
        fl = QFormLayout()
        self.cmb_start = QComboBox()
        self.cmb_end = QComboBox()
        self.spin_time = QSpinBox(); self.spin_time.setRange(0, 1440); self.spin_time.setValue(480) 
        self.spin_time.setSuffix(" min (Time of Day)")
        self.spin_budget = QSpinBox(); self.spin_budget.setRange(0, 10000); self.spin_budget.setValue(50); self.spin_budget.setSuffix(" $")
        
        fl.addRow("Origin:", self.cmb_start)
        fl.addRow("Destination:", self.cmb_end)
        fl.addRow("Start Time:", self.spin_time)
        fl.addRow("Max Budget:", self.spin_budget)
        grp_trip.setLayout(fl)

        grp_sets = QGroupBox("Settings & Objective")
        vl = QVBoxLayout()
        
        hl_t = QHBoxLayout()
        hl_t.addWidget(QLabel("Traffic Factor:"))
        self.spin_traffic = QDoubleSpinBox(); self.spin_traffic.setRange(1.0, 5.0); self.spin_traffic.setValue(1.0); self.spin_traffic.setSingleStep(0.1)
        hl_t.addWidget(self.spin_traffic)
        vl.addLayout(hl_t)
        hl_m = QHBoxLayout(); 
        self.chk_metro = QCheckBox("Metro"); self.chk_metro.setChecked(True)
        self.chk_bus = QCheckBox("Bus"); self.chk_bus.setChecked(True)
        self.chk_taxi = QCheckBox("Taxi"); self.chk_taxi.setChecked(True)
        self.chk_walk = QCheckBox("Walk"); self.chk_walk.setChecked(True)
        for c in [self.chk_metro, self.chk_bus, self.chk_taxi, self.chk_walk]: hl_m.addWidget(c)
        vl.addLayout(hl_m)

        self.cmb_algo = QComboBox(); self.cmb_algo.addItems(["Dijkstra (Optimal)", "A* Search (Heuristic)", "BFS (Min Hops)"])
        self.cmb_obj = QComboBox()
        self.cmb_obj.addItems(["Fastest Time", "Cheapest Cost", "Least Walking"])
        
        vl.addWidget(QLabel("Algorithm:"))
        vl.addWidget(self.cmb_algo)
        vl.addWidget(QLabel("Optimization Basis:"))
        vl.addWidget(self.cmb_obj)
        grp_sets.setLayout(vl)

        btn_calc = QPushButton("FIND BEST ROUTE")
        btn_calc.clicked.connect(self.calculate_route)
        btn_calc.setStyleSheet("background-color: #00b894; font-weight: bold; padding: 12px;")

        self.lbl_result = QLabel("Ready.")
        self.lbl_result.setAlignment(Qt.AlignCenter)
        self.lbl_result.setStyleSheet("color: #fab1a0; font-weight: bold;")

        self.table_res = QTableWidget()
        self.table_res.setColumnCount(4)
        self.table_res.setHorizontalHeaderLabels(["From->To", "Type", "Wait/Ride", "Arr Time"])
        self.table_res.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table_res.verticalHeader().setVisible(False)

        layout.addWidget(grp_trip)
        layout.addWidget(grp_sets)
        layout.addWidget(btn_calc)
        layout.addWidget(self.lbl_result)
        layout.addWidget(self.table_res)
    
    def setup_group_tab(self):
        layout = QVBoxLayout(self.tab_group)
        
        grp_add = QGroupBox("Add Traveler")
        fl = QFormLayout()
        self.txt_g_name = QLineEdit()
        self.cmb_g_start = QComboBox()
        self.spin_g_bud = QSpinBox(); self.spin_g_bud.setValue(50)
        btn_add = QPushButton("Add")
        btn_add.clicked.connect(self.add_traveler)
        fl.addRow("Name:", self.txt_g_name)
        fl.addRow("Start Node:", self.cmb_g_start)
        fl.addRow("Budget:", self.spin_g_bud)
        fl.addRow(btn_add)
        grp_add.setLayout(fl)

        self.list_g = QListWidget()

        grp_dest = QGroupBox("Meeting Point")
        fl2 = QFormLayout()
        self.cmb_g_meet = QComboBox()
        self.spin_g_time = QSpinBox(); self.spin_g_time.setRange(0, 1440); self.spin_g_time.setValue(600)
        fl2.addRow("Meet At:", self.cmb_g_meet)
        fl2.addRow("Global Time:", self.spin_g_time)
        grp_dest.setLayout(fl2)

        btn_solve_g = QPushButton("CALCULATE GROUP LOGISTICS")
        btn_solve_g.clicked.connect(self.solve_group)
        btn_solve_g.setStyleSheet("background-color: #6c5ce7;")

        self.table_g_res = QTableWidget()
        self.table_g_res.setColumnCount(3)
        self.table_g_res.setHorizontalHeaderLabels(["Name", "Duration", "Cost"])
        self.table_g_res.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        layout.addWidget(grp_add)
        layout.addWidget(self.list_g)
        layout.addWidget(grp_dest)
        layout.addWidget(btn_solve_g)
        layout.addWidget(self.table_g_res)