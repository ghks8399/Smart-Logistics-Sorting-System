#yolov8n 학습 시키는 코드
from roboflow import Roboflow
from ultralytics import YOLO

def main():
    rf = Roboflow(api_key="A3m3tbZoFbl0YcA9VTga")
    project = rf.workspace("datas-yn51k").project("box_vynli")
    version = project.version(6)
    dataset = version.download("yolov8")
                    
    model = YOLO("yolov8n.pt")
    results = model.train(
        data=f"{dataset.location}/data.yaml",
        epochs=150,
        imgsz=640,
        batch=16,
        patience=30,
        project="parcel_defect",
        name="yolov8n_v1",
    )

    metrics = model.val()
    print(metrics.box.map)
    print(metrics.box.map50)

    results = model.predict(
        source=f"{dataset.location}/test/images",
        conf=0.25,
        save=True,
    )

if __name__ == "__main__":
    main()