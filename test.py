import RPi.GPIO as GPIO
import time
import paho.mqtt.client as mqtt

# GPIO 핀 설정
motor_pin1 = 17  # IN1
motor_pin2 = 27  # IN2
pwm_pin = 18     # ENA (PWM 제어 핀)
trig_pin = 23    # 초음파 센서 트리거 핀
echo_pin = 24    # 초음파 센서 에코 핀
buzzer_pin = 22  # 부저 핀

# 임계값 설정
distance_threshold = 10  # 거리 임계값 (단위: cm)

# GPIO 초기화
GPIO.setmode(GPIO.BCM)
GPIO.setup(motor_pin1, GPIO.OUT)
GPIO.setup(motor_pin2, GPIO.OUT)
GPIO.setup(pwm_pin, GPIO.OUT)
GPIO.setup(trig_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)
GPIO.setup(buzzer_pin, GPIO.OUT)

# PWM 설정 (주파수 100Hz)
pwm = GPIO.PWM(pwm_pin, 100)
pwm.start(0)  # 초기 속도 0%

# MQTT 클라이언트 설정
def on_connect(client, userdata, flags, rc):
    print("서버가 브로커에 연결되었습니다.")
    client.subscribe("vector/optimal_direction")  # 벡터 데이터 구독

def on_message(client, userdata, msg):
    vector_data = msg.payload.decode()  # 벡터 데이터를 수신
    try:
        # 수신된 데이터를 처리하여 모터 제어 함수 호출
        x_value = float(vector_data.split(',')[0].split(":")[1].strip())  # x 값 추출
        set_motor(x_value)
        print(f"수신된 벡터로 모터 제어: {x_value}")
    except Exception as e:
        print(f"벡터 데이터 처리 중 오류 발생: {e}")

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

broker_ip = '103.218.162.56'  # 서버의 공인 IP (MQTT 브로커가 실행되는 IP)
client.connect(broker_ip, 2934, 60)

# 초음파 센서로 거리 측정
def measure_distance():
    GPIO.output(trig_pin, GPIO.HIGH)
    time.sleep(0.00001)
    GPIO.output(trig_pin, GPIO.LOW)

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        end_time = time.time()

    elapsed_time = end_time - start_time
    distance = (elapsed_time * 34300) / 2
    return distance

# 모터 제어 함수
def set_motor(x_value):
    if x_value > 0:
        GPIO.output(motor_pin1, GPIO.HIGH)
        GPIO.output(motor_pin2, GPIO.LOW)  # 시계방향
        direction = "시계"
    else:
        GPIO.output(motor_pin1, GPIO.LOW)
        GPIO.output(motor_pin2, GPIO.HIGH)  # 반시계방향
        direction = "반시계"
    
    pwm_value = min(max(abs(x_value) * 100, 0), 100)
    pwm.ChangeDutyCycle(pwm_value)
    print(f"방향: {direction}, pwm 값: {pwm_value:.2f}%")

# MQTT 클라이언트를 사용한 메인 로직
def main():
    try:
        while True:
            distance = measure_distance()
            print(f"측정된 거리: {distance:.2f} cm")

            if distance < distance_threshold:
                GPIO.output(buzzer_pin, GPIO.HIGH)
                print("부저 작동: 물체가 너무 가깝습니다!")
            else:
                GPIO.output(buzzer_pin, GPIO.LOW)
            
            time.sleep(1)
    finally:
        pwm.stop()
        GPIO.cleanup()

if __name__ == "__main__":
    client.loop_start()  # MQTT 수신을 백그라운드에서 처리
    main()  # 메인 로직 실행
