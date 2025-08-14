import asyncio
import json
import ssl
import pymysql
import paho.mqtt.client as mqtt

# ----------------------------
# KONFIGURASI MQTT
# ----------------------------
LOCAL_BROKER = "localhost"
LOCAL_PORT = 1883
LOCAL_TOPIC = "DATA/LOCAL/SENSOR/PANEL_1"

CLOUD_BROKER = "5de3065f0ebb48d986135895199984d6.s2.eu.hivemq.cloud"
CLOUD_PORT = 8883
CLOUD_TOPIC = "DATA/ONLINE/SENSOR/PANEL_1"
CLOUD_USERNAME = "embedded_test"
CLOUD_PASSWORD = "Ravelware1402"

# ----------------------------
# KONFIGURASI DATABASE
# ----------------------------
db = pymysql.connect(
    host="localhost",
    user="iot_user",      # user baru yang sudah dibuat di MariaDB
    password="iot123",
    database="iot_monitoring"
)
cursor = db.cursor()

# ----------------------------
# EVENT LOOP & QUEUE
# ----------------------------
loop = asyncio.get_event_loop()
queue = asyncio.Queue()

# ----------------------------
# CALLBACK MQTT
# ----------------------------
def on_connect(client, userdata, flags, reasonCode, properties=None):
    client_id = client._client_id.decode() if isinstance(client._client_id, bytes) else client._client_id
    print(f"[{client_id}] Connected with code {reasonCode}")
    if client_id == "gateway_local":
        client.subscribe(LOCAL_TOPIC)
        print(f"[LOCAL] Subscribed to {LOCAL_TOPIC}")

def on_message(client, userdata, message):
    try:
        payload = json.loads(message.payload.decode())
        print("📥 Data diterima:", payload)

        # Simpan ke MySQL
        cursor.execute(
            "INSERT INTO iot_data (voltage, current, power, temperature) VALUES (%s, %s, %s, %s)",
            (payload.get('voltage'), payload.get('current'), payload.get('power'), payload.get('temperature'))
        )
        db.commit()

        # Masukkan ke queue async secara thread-safe
        loop.call_soon_threadsafe(queue.put_nowait, payload)

    except Exception as e:
        print("❌ Error processing message:", e)

# ----------------------------
# INIT CLIENT MQTT
# ----------------------------
client_local = mqtt.Client(client_id="gateway_local")
client_local.on_connect = on_connect
client_local.on_message = on_message

client_cloud = mqtt.Client(client_id="gateway_cloud")
client_cloud.username_pw_set(CLOUD_USERNAME, CLOUD_PASSWORD)
client_cloud.tls_set(cert_reqs=ssl.CERT_NONE)
client_cloud.tls_insecure_set(True)
client_cloud.on_connect = on_connect

# ----------------------------
# PUBLISH KE CLOUD
# ----------------------------
async def publish_to_cloud():
    while True:
        data = await queue.get()
        try:
            payload = json.dumps(data)
            result = client_cloud.publish(CLOUD_TOPIC, payload)
            result.wait_for_publish()
            print("📤 Data dikirim ke cloud.")
        except Exception as e:
            print("❌ Gagal kirim ke cloud, retrying...", e)
            await queue.put(data)
        await asyncio.sleep(0.1)

# ----------------------------
# START CLIENTS
# ----------------------------
def start_mqtt_clients():
    client_local.connect(LOCAL_BROKER, LOCAL_PORT)
    client_local.loop_start()

    client_cloud.connect(CLOUD_BROKER, CLOUD_PORT)
    client_cloud.loop_start()

# ----------------------------
# MAIN
# ----------------------------
async def main():
    start_mqtt_clients()
    await publish_to_cloud()

if __name__ == "__main__":
    try:
        loop = asyncio.get_event_loop()

        start_mqtt_clients()

        loop.create_task(publish_to_cloud())

        loop.run_forever()
    except KeyboardInterrupt:
        print("Program dihentikan.")
        client_local.loop_stop()
        client_cloud.loop_stop()
        db.close()

