from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QLabel, QSpinBox, QDoubleSpinBox,
    QComboBox, QPushButton, QTableWidget, QTableWidgetItem, QMessageBox,
    QCheckBox, QLineEdit
)
from PyQt5.QtGui import QIntValidator    
from PyQt5.QtCore import QTimer, Qt
import os
import csv
import subprocess
import signal

class UAVSimGUI(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("UAV Simülasyon Başlatıcı")
        self.clear_log_on_startup()
        self.sim_process = None
        self.rviz_process = None
        self.last_log_mod_time = None

        self.uav_count_spin = QSpinBox()
        self.uav_count_spin.setRange(1, 50)
        self.uav_count_spin.setValue(5)

        self.algorithm_combo = QComboBox()
        self.algorithm_combo.addItems([
            "1 - PSO - Particle Swarm Optimization",
            "2 - ACO - Ant Colony Optimization",
            "3 - SMA - Starling Murmuration Algorithm",
            "4 - GWO - Grey Wolf Optimizer",
            "5 - WOA - Whale Optimization Algorithm",
            "6 - CO - Cuckoo Optimization"
        ])

        self.formation_combo = QComboBox()
        self.formation_combo.addItems(["1 - Triangle", "2 - v Formation", "3 - Line","4 - Grid"])

        self.start_button = QPushButton("Simülasyonu Başlat")
        self.start_button.clicked.connect(self.start_simulation)

        self.stop_button = QPushButton("Simülasyonu Durdur")
        self.stop_button.clicked.connect(self.stop_simulation)

        self.rviz_button = QPushButton("RViz'i Aç")
        self.rviz_button.clicked.connect(self.start_rviz)

        self.rviz_stop_button = QPushButton("RViz'i Kapat")
        self.rviz_stop_button.clicked.connect(self.stop_rviz)

        self.coverage_spin = QDoubleSpinBox()
        self.coverage_spin.setRange(1.0, 100.0)
        self.coverage_spin.setValue(10.0)

        self.comm_spin = QDoubleSpinBox()
        self.comm_spin.setRange(1.0, 500.0)
        self.comm_spin.setValue(30.0)

        self.area_spin = QDoubleSpinBox()
        self.area_spin.setRange(50.0, 1000.0)
        self.area_spin.setValue(50.0)

        self.result_table = QTableWidget()
        self.result_table.setColumnCount(6)
        self.result_table.setHorizontalHeaderLabels([
            "Zaman (saniye)", "UAV Sayısı", "Algoritma", "Formasyon",
            "Kapsama (%)", "Alan Genişliği"
        ])
        self.repeat_checkbox = QCheckBox("Çoklu Dağılma kullan")
        self.iterations_edit = QLineEdit()
        self.iterations_edit.setValidator(QIntValidator(1, 10))   # yalnızca 1-10 arası rakam
        self.iterations_edit.setFixedWidth(60)
        self.iterations_edit.setEnabled(False)                    # varsayılan kapalı
        self.iterations_edit.setPlaceholderText("Tekrar")

        # checkbox aç/kapatınca textbox’ı da etkinleştir
        self.repeat_checkbox.toggled.connect(self.iterations_edit.setEnabled)

        layout = QVBoxLayout()
        layout.addWidget(QLabel("UAV Sayısı:"))
        layout.addWidget(self.uav_count_spin)
        layout.addWidget(QLabel("Algoritma:"))
        layout.addWidget(self.algorithm_combo)
        layout.addWidget(QLabel("Formasyon:"))
        layout.addWidget(self.formation_combo)
        layout.addWidget(self.start_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(self.rviz_button)
        layout.addWidget(self.rviz_stop_button)
        layout.addWidget(QLabel("Coverage Radius (m):"))
        layout.addWidget(self.coverage_spin)
        layout.addWidget(QLabel("Communication Range (m):"))
        layout.addWidget(self.comm_spin)
        layout.addWidget(QLabel("Alan Genişliği (m):"))
        layout.addWidget(self.area_spin)
        layout.addWidget(self.repeat_checkbox)
        layout.addWidget(self.iterations_edit)
        layout.addWidget(QLabel("Kapsama Sonuçları:"))
        layout.addWidget(self.result_table)

        self.setLayout(layout)

        self.coverage_timer = QTimer()
        self.coverage_timer.timeout.connect(self.check_for_coverage_update)
        self.coverage_timer.start(2000)

    def start_simulation(self):
        if self.sim_process and self.sim_process.poll() is None:
            os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
            self.sim_process.wait()

        ros_ws = os.path.expanduser("~/Desktop/Proje/ros2_ws")
        num_uav = self.uav_count_spin.value()
        algorithm = self.algorithm_combo.currentText().split(" - ")[0]
        formation = self.formation_combo.currentText().split(" - ")[0]
        coverage = self.coverage_spin.value()
        comm = self.comm_spin.value()
        area_size = self.area_spin.value()
        if self.repeat_checkbox.isChecked() and self.iterations_edit.text():
            iterations = int(self.iterations_edit.text())
        else:
            iterations = 1   # checkbox kapalıysa tek sefer
        try:
            build_cmd = f"cd '{ros_ws}' && colcon build"
            subprocess.run(["bash", "-c", build_cmd], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)

            launch_cmd = (
                f"cd '{ros_ws}' && source install/setup.bash && "
                f"ros2 launch uav_pack uav_simulation.launch.py "
                f"num_uav:={num_uav} algorithm:={algorithm} formation:={formation} "
                f"coverage_radius:={coverage} comm_threshold:={comm} area_size:={area_size} "
                f"dispersion_iters:={iterations}"  
            )
            self.sim_process = subprocess.Popen(
                ["bash", "-c", launch_cmd],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

            if not self.rviz_process or self.rviz_process.poll() is not None:
                rviz_config = os.path.join(ros_ws, "src/uav_pack/rviz/uav_config.rviz")
                rviz_cmd = f"cd '{ros_ws}' && source install/setup.bash && rviz2 -d '{rviz_config}'"
                self.rviz_process = subprocess.Popen(
                    ["bash", "-c", rviz_cmd],
                    preexec_fn=os.setsid,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )

            QMessageBox.information(self, "Başlatıldı", "Build ve simülasyon başlatıldı.")

        except subprocess.CalledProcessError as e:
            QMessageBox.critical(self, "Build Hatası", f"colcon build başarısız:\n{e}")
        except Exception as e:
            QMessageBox.critical(self, "Simülasyon Hatası", str(e))

    def clear_log_on_startup(self):
        """Program ilk açıldığında coverage_log.csv dosyasını sıfırlar."""
        log_path = os.path.expanduser(
            "./ros2_ws/coverage_log.csv"
        )
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            with open(log_path, "w", newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "Zaman (saniye)", "UAV Sayısı", "Algoritma", "Formasyon",
                    "Kapsama (%)", "Alan Genişliği"
                ])
            print("[INFO] Log dosyası sıfırlandı.")
        except Exception as e:
            print(f"[ERROR] Log dosyası sıfırlanamadı: {e}")
            

    def stop_simulation(self):
        try:
            if self.sim_process and self.sim_process.poll() is None:
                os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
            if self.rviz_process and self.rviz_process.poll() is None:
                os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGTERM)
            subprocess.call("pkill -f ros2", shell=True)
            QMessageBox.information(self, "Durduruldu", "Tüm ROS 2 ve RViz süreçleri sonlandırıldı.")
        except Exception as e:
            QMessageBox.critical(self, "Hata", f"Süreç sonlandırılamadı:\n{e}")

    def start_rviz(self):
        ros_ws = "/Proje/ros2_ws"
        rviz_config = os.path.join(ros_ws, "src/uav_pack/rviz/uav_config.rviz")
        if self.rviz_process and self.rviz_process.poll() is None:
            return
        command = f"cd '{ros_ws}' && source install/setup.bash && rviz2 -d '{rviz_config}'"
        try:
            self.rviz_process = subprocess.Popen(
                ["bash", "-c", command],
                preexec_fn=os.setsid,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            QMessageBox.information(self, "RViz", "RViz başlatıldı.")
        except Exception as e:
            QMessageBox.critical(self, "RViz Hatası", str(e))

    def stop_rviz(self):
        if self.rviz_process and self.rviz_process.poll() is None:
            os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGTERM)
            QMessageBox.information(self, "Kapatıldı", "RViz kapatıldı.")
        else:
            QMessageBox.warning(self, "Uyarı", "Aktif RViz süreci bulunamadı.")

    def load_coverage_results(self):
        log_path = os.path.expanduser("~/Desktop/Proje/ros2_ws/coverage_log.csv")

        self.result_table.setRowCount(0)
        try:
            with open(log_path, "r") as f:
                reader = csv.reader(f)
                headers = next(reader)
                for row_idx, row in enumerate(reader):
                    self.result_table.insertRow(row_idx)
                    for col_idx, val in enumerate(row):
                        item = QTableWidgetItem(str(val))
                        item.setTextAlignment(Qt.AlignCenter)
                        self.result_table.setItem(row_idx, col_idx, item)
        except FileNotFoundError:
            print("[WARN] coverage_log.csv bulunamadı, tablo boş gösteriliyor.")


    def check_for_coverage_update(self):
        log_path = os.path.expanduser("~/Desktop/Proje/ros2_ws/coverage_log.csv")

        if os.path.exists(log_path):
            mod_time = os.path.getmtime(log_path)
            if self.last_log_mod_time is None or mod_time > self.last_log_mod_time:
                self.last_log_mod_time = mod_time
                self.load_coverage_results()



    def closeEvent(self, event):
        try:
            if self.sim_process and self.sim_process.poll() is None:
                os.killpg(os.getpgid(self.sim_process.pid), signal.SIGTERM)
            if self.rviz_process and self.rviz_process.poll() is None:
                os.killpg(os.getpgid(self.rviz_process.pid), signal.SIGTERM)
        except Exception:
            pass
        event.accept()

if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    gui = UAVSimGUI()
    gui.resize(400, 600)
    gui.show()
    sys.exit(app.exec_())
