import pymysql

def get_db_connection():
    # MySQL 데이터베이스에 연결
    conn = pymysql.connect(
        host='localhost',
        port=3306,
        user='cafe_admin',
        password='cafe',
        db='RestaurantDB',
        charset='utf8'
    )
    return conn