import QtQuick
import QtQuick3D

Item {
    width: 600
    height: 400

    View3D {
        anchors.fill: parent

        environment: SceneEnvironment {
            clearColor: "#222222"
            backgroundMode: SceneEnvironment.Color
        }

        DirectionalLight {
            eulerRotation.x: -20
            brightness: 1.5
        }
        PerspectiveCamera {
                    // 1. Bajamos Z a 50 (bien cerca del centro 0,0,0)
                    z: 96
                    x: 0
                    y: 36
                    eulerRotation.x: -21
                    // 2. FUNDAMENTAL: Bajamos el clipNear casi a cero
                    // para que no recorte el robot al tenerlo en la cara
                    clipNear: 10
                    clipFar: 5000
                }

                Historial {
                    id: miRobot
                    objectName: "robotVisual"

                    // 3. Agrandamos el modelo 500 veces
                    scale: Qt.vector3d(500, 500, 500)
                }
    }
}
