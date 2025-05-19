
import sqlite3
from datetime import datetime, timedelta
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import os
import time
app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")  # CORS 허용
@app.route('/')
def index():
    return render_template('index.html')
@socketio.on('connect')
def handle_connect():
    print("클라이언트 연결됨")
    emit('message', {'data': '서버에 연결되었습니다.'})
@socketio.on('start_random')
def handle_start_random():
    print("랜덤 메시지 전송 시작")
    db_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../2. gradio-demos/pico.db')
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()
    last_seen_id = None  # 마지막으로 본 id
    while True:
        # 최근 12시간 데이터 조회 쿼리
        query = '''
        SELECT * FROM 피코
        ORDER BY datetime DESC LIMIT 1
        '''
        # 데이터 조회 실행
        cursor.execute(query)
        results = cursor.fetchall()
        print(results)
        
        if results:
            # 최근에 추가된 행의 ID 확인
            new_id = results[0][0]  # 첫 번째 컬럼은 id라고 가정
            if last_seen_id is None:
                last_seen_id = new_id  # 초기화
            if new_id > last_seen_id:
                # 새로운 행이 추가되었으면, 소켓 메시지 전송
                if results[0][3]==1:
                    data = '불량'
                else: 
                    data = '정상'
                
                emit('message', {'data': data}, broadcast=True)

                last_seen_id = new_id  # 마지막으로 본 id 업데이트
        time.sleep(5)  # 5초마다 데이터베이스를 확인
if __name__ == '__main__':
    socketio.run(app, host='0.0.0.0', port=5000, debug=True)    
