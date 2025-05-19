import sys
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *
import subprocess

class Ui_POS(object):
    def setupUi(self, POS):
        if not POS.objectName():
            POS.setObjectName(u"POS")
        POS.resize(600, 500)
        self.centralwidget = QWidget(POS)
        self.centralwidget.setObjectName(u"centralwidget")
        self.titleLabel = QLabel(self.centralwidget)
        self.titleLabel.setObjectName(u"titleLabel")
        self.titleLabel.setGeometry(QRect(200, 20, 200, 30))
        self.titleLabel.setAlignment(Qt.AlignCenter)
        self.textBrowser = QTextBrowser(self.centralwidget)
        self.textBrowser.setObjectName(u"textBrowser")
        self.textBrowser.setGeometry(QRect(60, 70, 480, 300))
        self.republishButton = QPushButton(self.centralwidget)
        self.republishButton.setObjectName(u"republishButton")
        self.republishButton.setGeometry(QRect(60, 400, 100, 30))
        self.resetButton = QPushButton(self.centralwidget)
        self.resetButton.setObjectName(u"resetButton")
        self.resetButton.setGeometry(QRect(180, 400, 100, 30))
        self.resetButton_2 = QPushButton(self.centralwidget)
        self.resetButton_2.setObjectName(u"resetButton_2")
        self.resetButton_2.setGeometry(QRect(300, 400, 100, 30))
        self.resetButton_3 = QPushButton(self.centralwidget)
        self.resetButton_3.setObjectName(u"resetButton_3")
        self.resetButton_3.setGeometry(QRect(420, 400, 100, 30))
        POS.setCentralWidget(self.centralwidget)

        self.retranslateUi(POS)

        QMetaObject.connectSlotsByName(POS)

    def retranslateUi(self, POS):
        POS.setWindowTitle(QCoreApplication.translate("POS", u"POS", None))
        self.titleLabel.setStyleSheet(QCoreApplication.translate("POS", u"font-size: 16px; font-weight: bold;", None))
        self.titleLabel.setText(QCoreApplication.translate("POS", u"\uc8fc\ubb38 \ub0b4\uc5ed", None))
        self.republishButton.setText(QCoreApplication.translate("POS", u"\uc7ac\ubc1c\ud589", None))
        self.resetButton.setText(QCoreApplication.translate("POS", u"\ub9ac\uc14b", None))
        self.resetButton_2.setText(QCoreApplication.translate("POS", u"\uc870\ud68c", None))
        self.resetButton_3.setText(QCoreApplication.translate("POS", u"\ud638\ucd9c", None))


class NODE(QThread, Node):
    message_received = Signal(str)

    def __init__(self, node_name='ros_subscriber_node'):
        QThread.__init__(self)
        Node.__init__(self, node_name)

        self.subscription = self.create_subscription(
            String, 'order', self.subscription_callback, 10)

        self.order_history = []

    def subscription_callback(self, msg):
        message = msg.data
        self.get_logger().info(f'Received order: {message}')
        self.order_history.append(message)

        if len(self.order_history) > 5:
            self.order_history.pop(0)

        self.message_received.emit(message)

    def get_last_orders(self):
        return self.order_history

    def get_order_position(self, order):
        if order in self.order_history:
            return self.order_history.index(order) + 1
        else:
            return -1


class GUI(QMainWindow, Ui_POS):
    def __init__(self, ros_thread):
        super().__init__()
        self.ros_thread = ros_thread
        self.ros_thread.message_received.connect(self.add_message)
        self.setupUi(self)
        self.setup_connections()

    def setup_connections(self):
        self.republishButton.clicked.connect(self.republish_last_orders)
        self.resetButton.clicked.connect(self.reset_text_browser)
        self.resetButton_2.clicked.connect(self.show_menu_window)
        self.resetButton_3.clicked.connect(self.show_robot_window)

    def add_message(self, message):
        existing_text = self.textBrowser.toPlainText()
        if not existing_text.startswith("Order received"):
            self.textBrowser.append("Order received:")

        self.textBrowser.append(f" - {message}")

    def republish_last_orders(self):
        last_orders = self.ros_thread.get_last_orders()
        if last_orders:
            self.textBrowser.append("\nLast 5 orders:")
            for order in last_orders:
                self.textBrowser.append(f" - {order}")
        else:
            self.textBrowser.append("\nNo orders to republish.")

    def reset_text_browser(self):
        self.textBrowser.clear()

    def show_menu_window(self):
        menu_window = QMainWindow(self)
        menu_window.setWindowTitle("조회 결과")
        menu_window.setGeometry(100, 100, 300, 200)

        daily_sales_button = QPushButton("일일매출", menu_window)
        daily_sales_button.setGeometry(50, 50, 200, 30)
        menu_sales_button = QPushButton("메뉴별매출", menu_window)
        menu_sales_button.setGeometry(50, 90, 200, 30)
        usage_pattern_button = QPushButton("이용형태", menu_window)
        usage_pattern_button.setGeometry(50, 130, 200, 30)

        daily_sales_button.clicked.connect(lambda: self.show_graph_window("일일매출"))
        menu_sales_button.clicked.connect(lambda: self.show_graph_window("메뉴별매출"))
        usage_pattern_button.clicked.connect(lambda: self.show_graph_window("이용형태"))

        menu_window.show()

    def show_graph_window(self, graph_type):
        graph_window = QMainWindow(self)
        graph_window.setWindowTitle(graph_type)
        graph_window.setGeometry(100, 100, 400, 300)

        label = QLabel(graph_window)
        label.setGeometry(10, 10, 380, 280)

        if graph_type == "일일매출":
            image_path = "/home/yujin/project_ws/src/test_node/test_node/test_node/img/daily_sales.png"
        elif graph_type == "메뉴별매출":
            image_path = "/home/yujin/project_ws/src/test_node/test_node/test_node/img/menu_sales.png"
        elif graph_type == "이용형태":
            image_path = "/home/yujin/project_ws/src/test_node/test_node/test_node/img/takeout_vs_dinein.png"
        else:
            image_path = None

        if image_path:
            pixmap = QPixmap(image_path)
            if not pixmap.isNull():
                label.setPixmap(pixmap)
                label.setScaledContents(True)
            else:
                label.setText("이미지를 불러올 수 없습니다.")
                label.setAlignment(Qt.AlignCenter)
        else:
            label.setText("이미지 경로가 설정되지 않았습니다.")
            label.setAlignment(Qt.AlignCenter)

        graph_window.show()

    def show_robot_window(self):
        try:
            # subprocess를 통해 gui_test.py 실행
            subprocess.run(['python3', 'gui_test.py'], check=True)
        except Exception as e:
            print(f"Error while running gui_test.py: {e}")


def main():
    rclpy.init()
    node = NODE()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    gui = GUI(node)
    gui.show()

    try:
        sys.exit(app.exec_())
    except KeyboardInterrupt:
        sys.exit(0)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
