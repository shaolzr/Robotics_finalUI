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
def parse_command_with_llm(transcript_text):
    system_prompt = """
You are an AI agent interface. Convert the user's natural language command into a structured JSON format for downstream robotic execution.

Your output must only contain the JSON object, and should not include any explanation or extra text.

Use the following format:

{
  "intent": "fetch_item",
  "location": "Jack's office",
  "object": "a laptop",
  "destination": "sofa",
  "time": "now",
  "priority": "high"
}

end of JSON format.

Before concluding the text is not a valid command, double-check it carefully. If you are certain, then return your judgement along with the reasoning.
"""
# If the text is not like a command to robots, return "This input seems not to be a clear instruction. Please try again."
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
