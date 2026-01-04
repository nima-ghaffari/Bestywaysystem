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
                