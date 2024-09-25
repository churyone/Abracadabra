import paho.mqtt.client as mqtt
import time
import socket

broker_ip = '103.218.162.56'  # 서버의 공인 IP (MQTT 브로커가 실행되는 IP)

def get_ip_address():
    return socket.gethostbyname(socket.gethostname())

# MQTT 클라이언트 설정
client = mqtt.Client()

# 브로커에 연결
client.connect(broker_ip, 2934, 60)

while True:
    raspberry_ip = get_ip_address()
    # "heartbeat" 토픽에 현재 IP를 발행
    client.publish("heartbeat", raspberry_ip)
    print(f"하트비트 발행: {raspberry_ip}")
    time.sleep(5)  # 5초 간격으로 발행
