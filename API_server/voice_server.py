from fastapi import FastAPI, Form
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
import speech_recognition as sr
import pyttsx3
import threading
import queue
import os

app = FastAPI()

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

recognizer = sr.Recognizer()
mic = sr.Microphone()
tts_engine = pyttsx3.init()

# ✅ TTS queue to avoid concurrent runAndWait()
speech_queue = queue.Queue()


def tts_worker():
    while True:
        text = speech_queue.get()
        if text:
            print(f"🔊 Speaking via macOS say: {text}")
            try:
                # You can customize the voice: Alex, Victoria, etc.
                os.system(f'say "{text}"')  # Or add voice: say -v Alex "Hello"
            except Exception as e:
                print(f"❌ TTS error using 'say': {e}")
        speech_queue.task_done()

# ✅ Start the dedicated speech thread
threading.Thread(target=tts_worker, daemon=True).start()

@app.get("/listen")
def listen_for_voice():
    try:
        with mic as source:
            recognizer.adjust_for_ambient_noise(source)
            print("🎤 Listening...")
            audio = recognizer.listen(source, timeout=5)

        transcript = recognizer.recognize_google(audio)
        print(f"🧠 Transcript: {transcript}")
        return {"transcript": str(transcript)}

    except sr.WaitTimeoutError:
        return {"transcript": ""}
    except sr.UnknownValueError:
        return {"transcript": ""}
    except Exception as e:
        return {"error": str(e)}

@app.post("/speak")
def speak_out(text: str = Form(...)):
    print(f"🔁 Queued speech: {text}")
    speech_queue.put(text)
    return {"status": "queued"}

if __name__ == "__main__":
    uvicorn.run("voice_server:app", host="0.0.0.0", port=8000, reload=False)