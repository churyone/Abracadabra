import paho.mqtt.client as mqtt
import time

vector_data = [1.0, 2.0, 3.0]  # 전송할 벡터 데이터

# MQTT 클라이언트 설정
def on_connect(client, userdata, flags, rc):
    print("서버가 브로커에 연결되었습니다.")
    client.subscribe("heartbeat")  # 하트비트 토픽 구독

def on_message(client, userdata, msg):
    raspberry_ip = msg.payload.decode()
    print(f"수신된 하트비트 IP: {raspberry_ip}")

    # "vector" 토픽에 벡터 데이터 발행
    message = f"Vector data: {vector_data}"
    client.publish(f"vector/{raspberry_ip}", message)
    print(f"벡터 데이터 발행: {message}")

client = mqtt.Client()

client.on_connect = on_connect
client.on_message = on_message

broker_ip = '103.218.162.56'  # 서버의 공인 IP (MQTT 브로커가 실행되는 IP)
# broker_ip = '192.168.1.132'  # 서버의 공인 IP (MQTT 브로커가 실행되는 IP)

# 브로커에 연결
client.connect(broker_ip, 2934, 60)

client.loop_forever()  # 계속해서 메시지를 기다림
