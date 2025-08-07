import asyncio
import json
import ssl
import pymysql
import paho.mqtt.client as mqtt

# Konfigurasi MQTT lokal (node ➝ gateway)
LOCAL_BROKER = "localhost"
LOCAL_PORT = 1883
LOCAL_TOPIC = "DATA/LOCAL/SENSOR/PANEL_1"

# Konfigurasi MQTT cloud (gateway ➝ server)
CLOUD_BROKER = "5de3065f0ebb48d986135895199984d6.s2.eu.hivemq.cloud"
CLOUD_PORT = 8883
CLOUD_TOPIC = "DATA/ONLINE/SENSOR/PANEL_1"
CLOUD_USERNAME = "embedded_test"
CLOUD_PASSWORD = "Ravelware1402"

# Koneksi ke MySQL
db = pymysql.connect(
    host="localhost",
    user="root",
    password="password",
    database="iot_monitoring"
)
cursor = db.cursor()

client_local = mqtt.Client("gateway_local")
client_cloud = mqtt.Client("gateway_cloud")

client_cloud.username_pw_set(CLOUD_USERNAME, CLOUD_PASSWORD)
client_cloud.tls_set(cert_reqs=ssl.CERT_NONE)
client_cloud.tls_insecure_set(True)

queue = asyncio.Queue()

def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode())
        print("📥 Data diterima:", payload)

        cursor.execute(
            "INSERT INTO iot_data (voltage, current, power, temperature) VALUES (%s, %s, %s, %s)",
            (payload['voltage'], payload['current'], payload['power'], payload['temperature'])
        )
        db.commit()

        asyncio.run_coroutine_threadsafe(queue.put(payload), asyncio.get_event_loop())

    except Exception as e:
        print("❌ Error:", e)

async def publish_to_cloud():
    while True:
        data = await queue.get()
        try:
            payload = json.dumps(data)
            result = client_cloud.publish(CLOUD_TOPIC, payload)
            result.wait_for_publish()
            print("📤 Data dikirim ke cloud.")
        except Exception as e:
            print("❌ Gagal kirim ke cloud, retrying...")
            await queue.put(data)  # Masukkan kembali ke queue
        await asyncio.sleep(1)

def start_mqtt_clients():
    client_local.on_message = on_message
    client_local.connect(LOCAL_BROKER, LOCAL_PORT)
    client_local.subscribe(LOCAL_TOPIC)
    client_local.loop_start()

    client_cloud.connect(CLOUD_BROKER, CLOUD_PORT)
    client_cloud.loop_start()

async def main():
    start_mqtt_clients()
    await publish_to_cloud()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program dihentikan.")
        client_local.loop_stop()
        client_cloud.loop_stop()
        db.close()
