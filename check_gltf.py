import json

with open("c:/Users/hp/Downloads/Robotic arm/Robotic Arm 3D Model.gltf", "r", encoding="utf-8", errors="ignore") as f:
    try:
        data = json.load(f)
        print("Keys:", list(data.keys()))
        if "buffers" in data:
            print("Buffers:", data["buffers"])
        if "images" in data:
            print("Images:", data["images"])
    except Exception as e:
        print("Error parsing JSON:", e)
