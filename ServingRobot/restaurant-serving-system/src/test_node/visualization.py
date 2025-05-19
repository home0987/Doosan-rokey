import pymysql
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib import rc
from db_connection import get_db_connection  # Use the existing DB connection file

# Set up Korean font (NanumGothic)
plt.rc('font', family='NanumGothicCoding')
# Fetch data from the database
def get_db_data():
    connection = get_db_connection()  # Call external connection function
    query = "SELECT * FROM orders;"
    df = pd.read_sql(query, connection)  # Convert to a DataFrame
    connection.close()
    return df

# Visualization functions
def plot_daily_sales(df):
    """일일 매출표 (선 그래프)"""
    df['order_date'] = pd.to_datetime(df['order_time']).dt.date  # 날짜 추출
    daily_sales = df.groupby('order_date')['price'].sum()  # 일별 매출 합계

    # Convert to NumPy arrays
    x = daily_sales.index.to_numpy()  # Dates
    y = daily_sales.values  # Sales amounts

    plt.figure(figsize=(10, 6))
    plt.plot(x, y, marker='o')
    plt.title('일일 매출', fontsize=16)
    plt.xlabel('날짜', fontsize=12)
    plt.ylabel('총 매출 (₩)', fontsize=12)
    plt.grid(True)
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig('img/daily_sales.png')  # Save as image
    print("일일 매출표 저장 완료: daily_sales.png")

def plot_menu_sales(df):
    """메뉴별 매출 (파이 그래프)"""
    menu_sales = df.groupby('menu')['price'].sum()  # 메뉴별 매출 합계

    plt.figure(figsize=(8, 8))
    plt.pie(menu_sales.values, labels=menu_sales.index, autopct='%1.1f%%', startangle=140)
    plt.title('메뉴별 매출 분포', fontsize=16)
    plt.tight_layout()
    plt.savefig('img/menu_sales.png')  # Save as image
    print("메뉴별 매출 저장 완료: menu_sales.png")

def plot_takeout_vs_dinein(df):
    """포장 vs 매장 비교 (막대 그래프)"""
    takeout_sales = df.groupby('takeout')['price'].sum()  # 포장/매장별 매출 합계
    takeout_labels = ['매장', '포장'] if '매장' in takeout_sales.index else takeout_sales.index  # Label adjustment

    plt.figure(figsize=(8, 6))
    plt.bar(takeout_labels, takeout_sales.values, color=['blue', 'orange'])
    plt.title('포장 vs 매장 매출', fontsize=16)
    plt.xlabel('주문 유형', fontsize=12)
    plt.ylabel('총 매출 (₩)', fontsize=12)
    plt.tight_layout()
    plt.savefig('img/takeout_vs_dinein.png')  # Save as image
    print("포장 vs 매장 비교 저장 완료: takeout_vs_dinein.png")

# Main execution
def main():
    # Fetch data
    df = get_db_data()

    # Generate each visualization
    plot_daily_sales(df)
    plot_menu_sales(df)
    plot_takeout_vs_dinein(df)

if __name__ == "__main__":
    main()
