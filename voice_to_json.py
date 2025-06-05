import openai
import sounddevice as sd
import numpy as np
from scipy.io.wavfile import write
import os
from datetime import datetime
import threading

# === OpenAI client ===
client = openai.OpenAI(
    api_key=os.getenv("OPENAI_API_KEY")
)

SAMPLE_RATE = 44100

# ==== Global vars ====
recording = []
recording_stream = None
timestamp = None
audio_path = None

# ==== Utility ====
def get_timestamp():
    return datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

# ==== Audio Recording ====
def start_recording():
    global recording, recording_stream, timestamp
    recording = []
    timestamp = get_timestamp()

    def callback(indata, frames, time_info, status):
        if status:
            print(status)
        recording.append(indata.copy())

    try:
        recording_stream = sd.InputStream(
            samplerate=SAMPLE_RATE,
            channels=1,
            callback=callback,
            device=0,  # 使用默认设备
            blocksize=1024
        )
        recording_stream.start()
        print("🎙️ Recording started...")
    except Exception as e:
        print(f"Error starting recording: {str(e)}")
        return None


def stop_recording():
    global audio_path, recording_stream, recording, timestamp
    if recording_stream:
        recording_stream.stop()
        recording_stream.close()
        audio_data = np.concatenate(recording, axis=0)
        audio_path = f"recorded_audio_{timestamp}.wav"
        write(audio_path, SAMPLE_RATE, audio_data)
        print(f"✅ Recording saved: {audio_path}")
        return audio_path, timestamp
    else:
        print("⚠️ No active recording.")
        return None, None

# ==== Transcribe ====
def transcribe_audio(file_path):
    print("📤 Uploading and transcribing...")
    with open(file_path, "rb") as f:
        response = client.audio.transcriptions.create(
            model="gpt-4o-transcribe",
            file=f,
            response_format="text"
        )
    return response

# ==== LLM parse ====
def parse_command_with_llm(transcript_text, context=None):
    system_prompt = """
You are an AI agent interface. Your persona is a friendly and helpful farm worker. Use informal language and farm-related expressions where appropriate.
Analyze the user's input and determine if it's a command to execute a task or a status query.
If it's a command to execute a task, convert it into a structured JSON format for downstream robotic execution.
If it's a status query, return a JSON object indicating it's a query and directly provide the answer in English in the 'answer' field. Speak like a farm worker.
For commands, use this format:
{
  "type": "command",
  "object": "object_name",
  "destination": "destination_name"
}
Destination must be one of: "Mickey's House", "Minnie's Bontique", "Pluto's Den". Any similar words should be seen as these.
For status queries, use this format:
{
  "type": "query",
  "answer": "<your answer in English>"
}
Quesitons inlude asking distance, ETA, available objects, etc. Use the context I give to you in the begining.
If the command cannot be parsed into the command format (missing object or destination), return:
{
  "type": "error",
  "error": "Invalid command format. Please specify both an object to fetch and a destination."
}
If the user asks for an object and it's not in the 'Available objects' list in the context, respond in your farm worker persona that you don't see that item around the farm right now, or that the barn's runnin' low on that.
"""
    if context:
        system_prompt += f"\nHere is the latest robot status:\n{context}\n"
    response = client.chat.completions.create(
        model="gpt-4",
        messages=[
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": transcript_text}
        ]
    )
    return response.choices[0].message.content

# ==== Save Outputs ====
def save_outputs(transcript, structured_json, timestamp):
    transcript_file = f"transcription_{timestamp}.txt"
    json_file = f"structured_command_{timestamp}.json"

    with open(transcript_file, "w", encoding="utf-8") as f:
        f.write(transcript)
    with open(json_file, "w", encoding="utf-8") as f:
        f.write(structured_json)

    print(f"💾 Saved transcript: {transcript_file}")
    print(f"💾 Saved JSON: {json_file}")

# ==== Pipeline for threading ====
def process_pipeline(audio_path, timestamp, callback=None):
    try:
        transcript = transcribe_audio(audio_path)
        structured = parse_command_with_llm(transcript)
        save_outputs(transcript, structured, timestamp)
        if callback:
            callback(transcript, structured)
        return transcript, structured
    except Exception as e:
        print("❌ ", str(e))
        if callback:
            callback(None, None, error=str(e))
        return None, None

def threaded_process(audio_path, timestamp, callback=None):
    thread = threading.Thread(target=process_pipeline, args=(audio_path, timestamp, callback))
    thread.start()
