import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtWidgets import *
import subprocess  # order.py 실행을 위한 모듈
from db_connection import get_db_connection


class ROSNode(Node):
    def __init__(self):
        super().__init__('table_gui_node')
        self.publisher_ = self.create_publisher(String, 'order_topic', 10)
        self.get_logger().info("ROS Node is up and running")

        # 테이블별 주문 내역 #순서확인관련, 지워도 됨!
        self.order_history = []

    def publish_order(self, order_type):
        msg = String()
        msg.data = order_type
        self.publisher_.publish(msg)
        self.get_logger().info(f"Order Published: {order_type}")

        # 받은 주문을 order_history에 추가
        self.order_history.append(order_type)

    def get_order_position(self, order_type):
        # 주문이 목록에 있으면 순서를 반환
        if order_type in self.order_history:
            return self.order_history.index(order_type) + 1  # 1-based index
        else:
            return -1  # 주문이 목록에 없으면 -1 반환


class Ui_MainWindow(object):
    def setupUi(self, MainWindow, ros_node):
        self.ros_node = ros_node
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(410, 373)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        
        self.pushButton = QPushButton(self.centralwidget)
        self.pushButton.setObjectName(u"pushButton")
        self.pushButton.setGeometry(QRect(60, 160, 101, 25))
        
        self.pushButton_2 = QPushButton(self.centralwidget)
        self.pushButton_2.setObjectName(u"pushButton_2")
        self.pushButton_2.setGeometry(QRect(220, 160, 101, 25))
        
        self.pushButton_3 = QPushButton(self.centralwidget)
        self.pushButton_3.setObjectName(u"pushButton_3")
        self.pushButton_3.setGeometry(QRect(220, 200, 101, 25))
        
        self.pushButton_4 = QPushButton(self.centralwidget)
        self.pushButton_4.setObjectName(u"pushButton_4")
        self.pushButton_4.setGeometry(QRect(60, 200, 101, 25))
        
        self.label = QLabel(self.centralwidget)
        self.label.setObjectName(u"label")
        self.label.setGeometry(QRect(160, 80, 67, 17))
        
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName(u"menubar")
        self.menubar.setGeometry(QRect(0, 0, 410, 28))
        MainWindow.setMenuBar(self.menubar)
        
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName(u"statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        # 버튼 클릭 시 동작 연결
        self.pushButton.clicked.connect(lambda: self.open_order_window("포장하기"))
        self.pushButton_2.clicked.connect(lambda: self.open_order_window("먹고가기"))
        self.pushButton_3.clicked.connect(self.show_calling_window)
        self.pushButton_4.clicked.connect(self.show_order_status_window)

        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"table", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"포장하기", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"먹고가기", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"반납하기", None))
        self.pushButton_4.setText(QCoreApplication.translate("MainWindow", u"순서확인", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"1번 테이블", None))

    def open_order_window(self, order_type):
        print(f"{order_type} 버튼 클릭됨!")
        self.ros_node.publish_order(order_type)
        self.run_order_script(order_type)
        self.close_main_window()

    def run_order_script(self, order_type):
        # subprocess를 별도의 스레드에서 실행
        thread = threading.Thread(target=self.run_order_script_in_thread, args=(order_type,))
        thread.start()

    def run_order_script_in_thread(self, order_type): # 포장하기, 먹고가기 선택 
        try:
            subprocess.run(['python3', 'order.py', order_type, '1'])  # 'order.py' 실행
        except Exception as e:
            print(f"Error while running the order script: {e}")

    def close_main_window(self):
        QApplication.quit()

    def show_calling_window(self): # 반납하기 선택 
        try:
            # 현재 창 닫기
            QApplication.instance().activeWindow().close()
            # subprocess를 통해 gui_test.py 실행
            subprocess.run(['python3', 'gui_test.py'], check=True) # 'gui_test.py' 실행
            QApplication.quit()
        except Exception as e:
            print(f"Error while running gui_test.py: {e}")

    def show_order_status_window(self): 
        order_status_window = QDialog()
        order_status_window.setWindowTitle("순서 확인")
        order_status_window.setGeometry(100, 100, 400, 200)

        # 주문의 순서 확인
        order_types = ["포장하기", "매장식사"]
        positions = []

        for order_type in order_types:
            position = self.ros_node.get_order_position(order_type)
            positions.append((order_type, position))

        # 결과가 -1이면 주문이 없다는 의미
        label_text = ""
        for order_type, position in positions:
            if position == -1:
                label_text += f"{order_type} 주문이 없습니다\n"
            else:
                label_text += f"{order_type} 주문은 {position}번째 순서입니다\n"

        label = QLabel(label_text, order_status_window)
        label.setGeometry(50, 50, 300, 100)

        order_status_window.exec_()

def ros_thread_function(ros_node):
    try:
        rclpy.spin(ros_node)
    except Exception as e:
        print(f"Error in ROS thread: {e}")


def main():
    rclpy.init()
    ros_node = ROSNode()

    # Start ROS 2 in a separate thread
    ros_thread = threading.Thread(target=ros_thread_function, args=(ros_node,), daemon=True)
    ros_thread.start()

    # PySide2 GUI Application
    app = QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow, ros_node)
    MainWindow.show()
    sys.exit(app.exec_())

    # Shutdown ROS 2 after the GUI is closed
    ros_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()