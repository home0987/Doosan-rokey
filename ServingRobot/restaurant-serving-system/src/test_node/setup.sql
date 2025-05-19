-- Database 생성
CREATE DATABASE IF NOT EXISTS RestaurantDB
CHARACTER SET utf8mb4
COLLATE utf8mb4_general_ci;

-- RestaurantDB 데이터베이스 사용
USE RestaurantDB;

-- orders 테이블 생성
CREATE TABLE IF NOT EXISTS orders (
    id INT AUTO_INCREMENT PRIMARY KEY,   -- 자동 증가 기본 키
    table_id INT NOT NULL,               -- 테이블 번호
    menu VARCHAR(255) NOT NULL,          -- 메뉴 이름
    price FLOAT NOT NULL,                -- 메뉴 가격
    takeout VARCHAR(10) NOT NULL,        -- 포장 여부 ('포장', '매장')
    order_time DATETIME NOT NULL         -- 주문 시간
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

