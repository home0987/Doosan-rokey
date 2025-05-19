import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtWidgets import *
import subprocess  # order.py 실행을 위한 모듈


class ROSNode(Node):
    def __init__(self):
        super().__init__('table_gui_node')
        self.publisher_ = self.create_publisher(String, 'order_topic', 10)
        self.get_logger().info("ROS Node is up and running")

    def publish_order(self, order_type):
        msg = String()
        msg.data = order_type
        self.publisher_.publish(msg)
        self.get_logger().info(f"Order Published: {order_type}")


class Ui_MainWindow(object):
    def setupUi(self, MainWindow, ros_node):
        self.ros_node = ros_node  # ROS Node reference
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
        self.pushButton_3.clicked.connect(lambda: self.show_calling_window())

        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"table", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"포장하기", None))
        self.pushButton_2.setText(QCoreApplication.translate("MainWindow", u"먹고가기", None))
        self.pushButton_3.setText(QCoreApplication.translate("MainWindow", u"반납하기", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"9번 테이블", None))

    def open_order_window(self, order_type):
        print(f"{order_type} 버튼 클릭됨!")
        self.ros_node.publish_order(order_type)
        self.run_order_script(order_type)
        self.close_main_window()

    def run_order_script(self, order_type):
        subprocess.run(['python3', 'order.py', order_type])  # 'order.py' 실행

    def close_main_window(self):
        QApplication.quit()

    def show_calling_window(self):

        calling_window = QDialog()
        calling_window.setWindowTitle("로봇 호출")
        calling_window.setGeometry(100, 100, 400, 300)
        
        label = QLabel("로봇을 호출합니다", calling_window)
        label.setGeometry(150, 100, 200, 30)
        
        print("로봇 호출 창 표시 중...")
        calling_window.exec_()


def ros_thread_function(ros_node):
    rclpy.spin(ros_node)


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
