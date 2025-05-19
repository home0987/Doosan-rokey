import subprocess
import sys
import threading

import pymysql
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from PySide2.QtCore import *
from PySide2.QtWidgets import *
from db_connection import get_db_connection
from datetime import datetime

class Ui_Form(QWidget):
    def __init__(self, ros_node, takeout, table_id):
        super().__init__()
        self.ros_node = ros_node
        self.selected_items = []  # 선택된 메뉴들을 저장할 리스트
        self.takeout = takeout  # "True" 또는 "False"
        self.table_id = table_id  # 테이블 번호
        self.total_price = 0  # 총 가격
        self.item_prices = {  # 메뉴와 가격 정보
            "아메리카노 (hot)": 4.0, "아메리카노 (ice)": 4.0,
            "카페라떼 (hot)": 4.5, "카페라떼 (ice)": 4.5,
            "바닐라라떼 (hot)": 5.5, "바닐라라떼 (ice)": 5.5, 
            "돌체라떼 (hot)": 5.0, "돌체라떼 (ice)": 5.0,
            "소금빵": 3.5, "와플": 3.0, "당근 케이크": 5.5
        }
        self.setupUi(self)

    def setupUi(self, Form):
        self.order_display = QTextEdit(Form)
        self.order_display.setGeometry(QRect(620, 47, 140, 250))
        self.order_display.setReadOnly(True)  # 편집 불가능
        self.order_display.setText("주문 내역:\n")

        # 총 가격 표시
        self.total_price_label = QLabel(Form)
        self.total_price_label.setGeometry(QRect(620, 295, 140, 25))
        self.total_price_label.setText(f"총 가격: {self.total_price:.1f}")

        Form.setObjectName("Form")
        Form.resize(773, 391)
        self.tabWidget = QTabWidget(Form)
        self.tabWidget.setGeometry(QRect(20, 20, 591, 331))

        # 음료 탭
        self.tab = QWidget()
        self.setup_beverage_tab(self.tab)
        self.tabWidget.addTab(self.tab, "음료")

        # 디저트 탭
        self.tab_2 = QWidget()
        self.setup_dessert_tab(self.tab_2)
        self.tabWidget.addTab(self.tab_2, "디저트")

        # 확인 버튼
        self.pushButton_6 = QPushButton(Form)
        self.pushButton_6.setGeometry(QRect(620, 330, 89, 25))
        self.pushButton_6.setText("확인")
        self.pushButton_6.clicked.connect(self.confirm_order)

        # 돌아가기 버튼
        self.back_button = QPushButton(Form)
        self.back_button.setGeometry(QRect(620, 20, 89, 25))
        self.back_button.setText("돌아가기")
        self.back_button.clicked.connect(self.back_to_table)

    def back_to_table(self):
        self.close()  # 현재 창을 닫고
        # 테이블 번호에 따라 해당 테이블 Python 파일을 실행합니다.
        if self.table_id == 1:
            subprocess.run(["python3", "table_1.py"])  # table_1.py 실행
        elif self.table_id == 2:
            subprocess.run(["python3", "table_2.py"])  # table_2.py 실행
        elif self.table_id == 3:
            subprocess.run(["python3", "table_3.py"])  # table_3.py 실행
        elif self.table_id == 4:
            subprocess.run(["python3", "table_4.py"])  # table_4.py 실행
        elif self.table_id == 5:
            subprocess.run(["python3", "table_5.py"])  # table_5.py 실행
        elif self.table_id == 6:
            subprocess.run(["python3", "table_6.py"])  # table_6.py 실행
        elif self.table_id == 7:
            subprocess.run(["python3", "table_7.py"])  # table_7.py 실행
        elif self.table_id == 8:
            subprocess.run(["python3", "table_8.py"])  # table_8.py 실행
        elif self.table_id == 9:
            subprocess.run(["python3", "table_9.py"])  # table_9.py 실행
        else:
            # 기본 테이블 처리 (예: 테이블이 추가될 수 있음)
            subprocess.run(["python3", "table_1.py"])  # 기본적으로 table_1.py 실행

    def setup_beverage_tab(self, tab):
        # 음료 버튼 및 옵션
        self.pushButton = QPushButton(tab)
        self.pushButton.setGeometry(QRect(20, 120, 89, 25))
        self.pushButton.setText("아메리카노")
        self.pushButton.clicked.connect(lambda: self.add_item("아메리카노"))

        self.radioButton = QRadioButton(tab)
        self.radioButton.setGeometry(QRect(20, 60, 112, 23))
        self.radioButton.setText("hot")
        self.radioButton_2 = QRadioButton(tab)
        self.radioButton_2.setGeometry(QRect(150, 60, 112, 23))
        self.radioButton_2.setText("ice")

        self.pushButton_3 = QPushButton(tab)
        self.pushButton_3.setGeometry(QRect(20, 220, 89, 25))
        self.pushButton_3.setText("카페라떼")
        self.pushButton_3.clicked.connect(lambda: self.add_item("카페라떼"))

        self.pushButton_4 = QPushButton(tab)
        self.pushButton_4.setGeometry(QRect(270, 120, 89, 25))
        self.pushButton_4.setText("바닐라라떼")
        self.pushButton_4.clicked.connect(lambda: self.add_item("바닐라라떼"))

        self.pushButton_5 = QPushButton(tab)
        self.pushButton_5.setGeometry(QRect(270, 220, 89, 25))
        self.pushButton_5.setText("돌체라떼")
        self.pushButton_5.clicked.connect(lambda: self.add_item("돌체라떼"))

        # 음료 라벨들
        self.label = QLabel(tab)
        self.label.setGeometry(QRect(20, 20, 241, 17))
        self.label.setText("음료를 선택해주세요")

        self.label_2 = QLabel(tab)
        self.label_2.setGeometry(QRect(130, 123, 67, 21))
        self.label_2.setText("4.0")

        self.label_3 = QLabel(tab)
        self.label_3.setGeometry(QRect(390, 123, 67, 21))
        self.label_3.setText("5.5")

        self.label_4 = QLabel(tab)
        self.label_4.setGeometry(QRect(130, 223, 67, 21))
        self.label_4.setText("4.5")

        self.label_5 = QLabel(tab)
        self.label_5.setGeometry(QRect(390, 223, 67, 21))
        self.label_5.setText("5.0")

    def setup_dessert_tab(self, tab_2):
        # 디저트 버튼
        self.pushButton_2 = QPushButton(tab_2)
        self.pushButton_2.setGeometry(QRect(20, 30, 89, 25))
        self.pushButton_2.setText("소금빵")
        self.pushButton_2.clicked.connect(lambda: self.add_item("소금빵"))

        self.pushButton_7 = QPushButton(tab_2)
        self.pushButton_7.setGeometry(QRect(20, 130, 89, 25))
        self.pushButton_7.setText("와플")
        self.pushButton_7.clicked.connect(lambda: self.add_item("와플"))

        self.pushButton_8 = QPushButton(tab_2)
        self.pushButton_8.setGeometry(QRect(20, 240, 89, 25))
        self.pushButton_8.setText("당근 케이크")
        self.pushButton_8.clicked.connect(lambda: self.add_item("당근 케이크"))

        # 디저트 라벨들
        self.label_6 = QLabel(tab_2)
        self.label_6.setGeometry(QRect(190, 33, 67, 21))
        self.label_6.setText("3.5")

        self.label_7 = QLabel(tab_2)
        self.label_7.setGeometry(QRect(190, 133, 67, 21))
        self.label_7.setText("3.0")

        self.label_8 = QLabel(tab_2)
        self.label_8.setGeometry(QRect(190, 243, 67, 21))
        self.label_8.setText("5.5")

    def add_item(self, item):
        # 음료에서만 온도 선택 확인
        if item in ["아메리카노", "카페라떼", "바닐라라떼", "돌체라떼"]:
            if not (self.radioButton.isChecked() or self.radioButton_2.isChecked()):
                QMessageBox.warning(self, "온도 선택 오류", "온도를 선택해주세요.")
                return  # 온도를 선택하지 않으면 메뉴 추가 안함

            # 음료 온도 정보 추가
            if self.radioButton.isChecked():
                item += " (hot)"
            elif self.radioButton_2.isChecked():
                item += " (ice)"
        
        self.selected_items.append(item)

        print(item)
        self.total_price += self.item_prices.get(item, 0) # 가격 추가
        
        self.update_order_display()

        print(f"Selected item: {item}")

    def update_order_display(self):
        """주문 내역과 총 가격을 업데이트"""
        self.order_display.setText("주문 내역:\n" + "\n".join(self.selected_items))
        self.total_price_label.setText(f"총 가격: {self.total_price:.1f}")


    def confirm_order(self):
        if not self.selected_items:
            error_message = "메뉴를 선택해주세요."
            QMessageBox.critical(self, "선택 오류", error_message)
        else:
            # 선택된 메뉴들과 테이블 정보를 퍼블리시
            order_msg = f"테이블 {self.table_id} - {self.takeout} 주문: " + ", ".join(self.selected_items)
            self.ros_node.publish_order(order_msg)
            print(f"Order published: {order_msg}")
            current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            # print('finish', self.takeout,self.table_id, self.selected_items)

            try:
                # 데이터베이스 연결
                connection = get_db_connection()
                with connection.cursor() as cursor:
                    # SQL 삽입 쿼리
                    sql = """
                    INSERT INTO orders (table_id, menu, price, takeout, order_time)
                    VALUES (%s, %s, %s, %s, %s)
                    """
                    # 각 메뉴를 개별 행으로 삽입
                    for item in self.selected_items:
                        cursor.execute(sql, (
                            self.table_id,          # 테이블 번호
                            item,                   # 메뉴 이름
                            self.item_prices.get(item, 0), # 메뉴 가격
                            self.takeout,         # 포장 여부 (문자열)
                            current_time            # 현재 시간
                        ))
                    connection.commit()
                    print("Order data inserted successfully!")
            except pymysql.MySQLError as e:
                print(f"Error inserting order data: {e}")
            finally:
                if connection:
                    connection.close()

            self.selected_items = []  # 주문 후, 리스트 비우기
            self.total_price = 0  # 총 가격 초기화
            self.update_order_display()
            self.ros_node.reset_order()

        print("주문 확인 완료")


# ROS Node Class
class RosNode(Node):
    def __init__(self):
        super().__init__('order_node')
        self.publisher = self.create_publisher(String, 'orders', 10)
        self.order_received = False

    def publish_order(self, msg):
        self.publisher.publish(String(data=msg))
        self.get_logger().info(f"Order published: {msg}")

    def reset_order(self):
        self.order_received = False


# Run ROS 2 and GUI in separate threads
def run_ros2(node):
    rclpy.spin(node)


def main():
    print(f"Arguments received: {sys.argv}")
    if len(sys.argv) < 3:
        print("사용법: python3 order.py <takeout_option> <table_id>")
        sys.exit(1)# Initialize ROS 2

    rclpy.init()
    ros_node = RosNode()
    print(sys.argv)
    # Get order type and table_id from command-line arguments
    takeout = '매장' if sys.argv[1] == '먹고가기' else '포장'
    table_id = sys.argv[2]    # 테이블 번호
    print(f"Takeout option: {takeout}, Table ID: {table_id}")

    # Run ROS 2 in a separate thread
    ros_thread = threading.Thread(target=run_ros2, args=(ros_node,), daemon=True)
    ros_thread.start()

    # Initialize PySide2 GUI
    app = QApplication(sys.argv)
    ui = Ui_Form(ros_node, takeout, table_id)  # Pass order type and table_id to the form
    ui.show()
    sys.exit(app.exec_())

    # Shutdown ROS 2
    rclpy.shutdown()


if __name__ == '__main__':
    main()