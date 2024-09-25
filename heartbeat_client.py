import paho.mqtt.client as mqtt
import socket

broker_ip = '103.218.162.56'  # 서버의 공인 IP (MQTT 브로커가 실행되는 IP)

# MQTT 클라이언트 설정
def on_connect(client, userdata, flags, rc):
    raspberry_ip = socket.gethostbyname(socket.gethostname())
    client.subscribe(f"vector/{raspberry_ip}")  # 자신의 IP에 해당하는 vector 토픽 구독
    print(f"vector/{raspberry_ip} 토픽을 구독 중...")

def on_message(client, userdata, msg):
    print(f"수신된 벡터 데이터: {msg.payload.decode()}")

client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message

# 브로커에 연결
client.connect(broker_ip, 2934, 60)

client.loop_forever()  # 계속해서 메시지를 기다림
