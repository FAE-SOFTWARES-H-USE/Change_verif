#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os  # Essencial para localizar o caminho do arquivo da logo
import serial
import serial.tools.list_ports
import time
import struct
from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QLabel, QComboBox, QPushButton, QTextEdit, QSpinBox, 
                             QGroupBox, QMessageBox, QFrame, QSizePolicy)
from PyQt5.QtCore import QThread, pyqtSignal, Qt
from PyQt5.QtGui import QTextCursor, QFont, QColor, QPalette, QPixmap # QPixmap adicionado aqui

# =============================================================================
# LÓGICA DO PROTOCOLO (Mantida conforme original)
# =============================================================================
CRC16_TABLE = [
    0x0000, 0x3d65, 0x7aca, 0x47af, 0xf594, 0xc8f1, 0x8f5e, 0xb23b, 0xd64d,
    0xeb28, 0xac87, 0x91e2, 0x23d9, 0x1ebc, 0x5913, 0x6476, 0x91ff, 0xac9a,
    0xeb35, 0xd650, 0x646b, 0x590e, 0x1ea1, 0x23c4, 0x47b2, 0x7ad7, 0x3d78,
    0x001d, 0xb226, 0x8f43, 0xc8ec, 0xf589, 0x1e9b, 0x23fe, 0x6451, 0x5934,
    0xeb0f, 0xd66a, 0x91c5, 0xaca0, 0xc8d6, 0xf5b3, 0xb21c, 0x8f79, 0x3d42,
    0x0027, 0x4788, 0x7aed, 0x8f64, 0xb201, 0xf5ae, 0xc8cb, 0x7af0, 0x4795,
    0x003a, 0x3d5f, 0x5929, 0x644c, 0x23e3, 0x1e86, 0xacbd, 0x91d8, 0xd677,
    0xeb12, 0x3d36, 0x0053, 0x47fc, 0x7a99, 0xc8a2, 0xf5c7, 0xb268, 0x8f0d,
    0xeb7b, 0xd61e, 0x91b1, 0xacd4, 0x1eef, 0x238a, 0x6425, 0x5940, 0xacc9,
    0x91ac, 0xd603, 0xeb66, 0x595d, 0x6438, 0x2397, 0x1ef2, 0x7a84, 0x47e1,
    0x004e, 0x3d2b, 0x8f10, 0xb275, 0xf5da, 0xc8bf, 0x23ad, 0x1ec8, 0x5967,
    0x6402, 0xd639, 0xeb5c, 0xacf3, 0x9196, 0xf5e0, 0xc885, 0x8f2a, 0xb24f,
    0x0074, 0x3d11, 0x7abe, 0x47db, 0xb252, 0x8f37, 0xc898, 0xf5fd, 0x47c6,
    0x7aa3, 0x3d0c, 0x0069, 0x641f, 0x597a, 0x1ed5, 0x23b0, 0x918b, 0xacee,
    0xeb41, 0xd624, 0x7a6c, 0x4709, 0x00a6, 0x3dc3, 0x8ff8, 0xb29d, 0xf532,
    0xc857, 0xac21, 0x9144, 0xd6eb, 0xeb8e, 0x59b5, 0x64d0, 0x237f, 0x1e1a,
    0xeb93, 0xd6f6, 0x9159, 0xac3c, 0x1e07, 0x2362, 0x64cd, 0x59a8, 0x3dde,
    0x00bb, 0x4714, 0x7a71, 0xc84a, 0xf52f, 0xb280, 0x8fe5, 0x64f7, 0x5992,
    0x1e3d, 0x2358, 0x9163, 0xac06, 0xeba9, 0xd6cc, 0xb2ba, 0x8fdf, 0xc870,
    0xf515, 0x472e, 0x7a4b, 0x3de4, 0x0081, 0xf508, 0xc86d, 0x8fc2, 0xb2a7,
    0x009c, 0x3df9, 0x7a56, 0x4733, 0x2345, 0x1e20, 0x598f, 0x64ea, 0xd6d1,
    0xebb4, 0xac1b, 0x917e, 0x475a, 0x7a3f, 0x3d90, 0x00f5, 0xb2ce, 0x8fab,
    0xc804, 0xf561, 0x9117, 0xac72, 0xebdd, 0xd6b8, 0x6483, 0x59e6, 0x1e49,
    0x232c, 0xd6a5, 0xebc0, 0xac6f, 0x910a, 0x2331, 0x1e54, 0x59fb, 0x649e,
    0x00e8, 0x3d8d, 0x7a22, 0x4747, 0xf57c, 0xc819, 0x8fb6, 0xb2d3, 0x59c1,
    0x64a4, 0x230b, 0x1e6e, 0xac55, 0x9130, 0xd69f, 0xebfa, 0x8f8c, 0xb2e9,
    0xf546, 0xc823, 0x7a18, 0x477d, 0x00d2, 0x3db7, 0xc83e, 0xf55b, 0xb2f4,
    0x8f91, 0x3daa, 0x00cf, 0x4760, 0x7a05, 0x1e73, 0x2316, 0x64b9, 0x59dc,
    0xebe7, 0xd682, 0x912d, 0xac48
]

CMD_CHANGE_OPMODE = 0x03
CMD_SYNC          = 0xA6
CMD_SYNC_CONN     = 0xA7
CMD_SYNC_ACK      = 0xA8
CMD_ACK           = 0xA9
CMD_NACK          = 0xAA
ACK_DATA_SIZE = 6

def calculate_crc(data, initial_crc=0):
    if isinstance(data, list): data = bytes(data)
    crc = initial_crc
    for byte in data:
        crc = CRC16_TABLE[byte ^ ((crc >> 8) & 0xFF)] ^ ((crc << 8) & 0xFFFF)
    temp = ~crc & 0xFFFF
    result_high_byte = (temp & 0xFF00) >> 8
    result_low_byte_shifted = (temp & 0x00FF) << 8
    return [result_low_byte_shifted >> 8, result_high_byte]

def assemble_pack(cmd_code, payload=b''):
    header_size = 4
    crc_size = 2
    total_size = header_size + len(payload) + crc_size
    size_low = total_size & 0xFF
    size_high = (total_size >> 8) & 0xFF
    header = [cmd_code, 0x01, size_low, size_high]
    data_to_crc = bytes(header) + payload
    crc_lsb, crc_msb = calculate_crc(data_to_crc)
    return data_to_crc + bytes([crc_msb, crc_lsb])

def format_hex(array):
    return " ".join([f"{b:02X}" for b in array])

# =============================================================================
# WORKER THREAD
# =============================================================================
class ComWorker(QThread):
    log_signal = pyqtSignal(str, str) # Mensagem, Cor
    finished_signal = pyqtSignal()

    def __init__(self, port, target_mode):
        super().__init__()
        self.port = port
        self.target_mode = target_mode
        self.access_key = 0x00000000
        self.running = True

    def log(self, text, color="#DDDDDD"):
        self.log_signal.emit(text, color)

    def send_mode_command(self, ser, mode):
        self.log(f"Solicitando troca para Modo: {mode}...", "#5DADE2")
        payload = struct.pack('<BI', mode, self.access_key)
        mode_packet = assemble_pack(CMD_CHANGE_OPMODE, payload)
        ser.reset_input_buffer()
        ser.write(mode_packet)
        for _ in range(15):
            if not self.running: return False
            resp = ser.read(ACK_DATA_SIZE)
            if resp:
                self.log(f"RX (RESP): {format_hex(resp)}")
                if resp[0] == CMD_ACK: return True
                elif resp[0] == CMD_NACK:
                    self.log(f"Erro: Medidor recusou o Modo {mode}.", "#DC3545")
                    return False
            time.sleep(0.1)
        return False

    def run(self):
        ser = None
        self.log(f"Iniciando conexão na {self.port}...", "#3498DB")
        try:
            ser = serial.Serial(self.port, 9600, timeout=1)
        except Exception as e:
            self.log(f"ERRO: Porta {self.port} ocupada ou inexistente.", "#DC3545")
            self.finished_signal.emit()
            return

        try:
            handshake_done = False
            self.log("[1/3] Sincronizando (Handshake)...", "#F1C40F")
            start_time = time.time()
            while (time.time() - start_time) < 10 and self.running:
                pack = ser.read(ACK_DATA_SIZE)
                if not pack: continue
                self.log(f"RX: {format_hex(pack)}")
                if pack[0] == CMD_SYNC:
                    self.log("SYNC (0xA6) recebido. Enviando ACK...")
                    ser.write(assemble_pack(CMD_SYNC_ACK))
                    for _ in range(10):
                        conf = ser.read(ACK_DATA_SIZE)
                        if conf and conf[0] == CMD_SYNC_CONN:
                            self.log("Conexão Estabelecida!", "#28A745")
                            handshake_done = True
                            break
                        time.sleep(0.1)
                elif pack[0] == CMD_SYNC_CONN:
                    handshake_done = True
                if handshake_done: break

            if handshake_done:
                time.sleep(0.3)
                success = True
                self.log(f"[2/2] Solicitando Modo {self.target_mode} diretamente...")
                success = self.send_mode_command(ser, self.target_mode)
                if success: 
                    self.log("PROCESSO FINALIZADO COM SUCESSO", "#28A745")
                else: 
                    self.log("FALHA NA TROCA DE MODO", "#DC3545")
            else:
                self.log("Falha: O medidor não respondeu.", "#DC3545")

        except Exception as e: self.log(f"Erro inesperado: {e}", "#DC3545")
        finally:
            if ser and ser.is_open: ser.close()
            self.log("Porta fechada.", "#6A737C")
            self.finished_signal.emit()


# =============================================================================
# INTERFACE GRÁFICA (Ajustada com Logo centralizada e Layout Horizontal)
# =============================================================================
class SerialApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("FCX Mode Optical")
        self.resize(600, 700) 
        self.apply_styles()
        self.init_ui()

    def apply_styles(self):
        dark_palette = QPalette()
        dark_palette.setColor(QPalette.Window, QColor(30, 34, 45))
        dark_palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
        dark_palette.setColor(QPalette.Base, QColor(40, 44, 55))
        dark_palette.setColor(QPalette.Text, QColor(220, 220, 220))
        dark_palette.setColor(QPalette.Button, QColor(45, 50, 65))
        dark_palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
        dark_palette.setColor(QPalette.Highlight, QColor(52, 152, 219))
        self.setPalette(dark_palette)
        
        self.setStyleSheet("""
            QWidget { color: #DDDDDD; font-size: 10pt; background-color: #1E222D; }
            QGroupBox {
                border: 1px solid #2C3E50;
                margin-top: 15px;
                padding: 15px;
                font-weight: bold;
                border-radius: 6px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                subcontrol-position: top center;
                padding: 0 10px;
                color: #5DADE2;
            }
            QPushButton {
                background-color: #3498DB;
                color: white;
                border: none;
                border-radius: 5px;
                padding: 8px;
                font-weight: bold;
            }
            QPushButton:hover { background-color: #5DADE2; }
            QComboBox, QSpinBox {
                border: 1px solid #3E4450;
                border-radius: 4px;
                padding: 5px;
                background-color: #2C2F3A;
                color: #DDDDDD;
            }
            QTextEdit {
                background-color: #2C2F3C;
                color: #EEEEEE;
                border: 1px solid #3E4450;
                border-radius: 5px;
                padding: 8px;
                font-family: 'Consolas', 'Monospace';
            }
        """)

    def init_ui(self):
        main_layout = QVBoxLayout(self)
        main_layout.setContentsMargins(20, 30, 20, 20)
        main_layout.setSpacing(15)

        # --- Logo Centralizada ---
        logo_label = QLabel()
        # Ajuste o caminho abaixo para onde sua logo está guardada
        logo_path = os.path.join(os.path.dirname(__file__), "assets", "logofae.png") 
        if os.path.exists(logo_path):
            pixmap = QPixmap(logo_path)
            logo_label.setPixmap(pixmap.scaledToHeight(80, Qt.SmoothTransformation))
        else:
            # Caso a imagem não exista, coloca apenas um texto centralizado
            logo_label.setText("FLUXUS MODE")
            logo_label.setStyleSheet("font-size: 18pt; font-weight: bold; color: #5DADE2;")
            
        logo_label.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(logo_label)

        main_layout.addSpacing(20)

        # --- Configuração Serial ---
        config_group = QGroupBox("Configuração Serial")
        config_lay = QHBoxLayout()
        self.port_combo = QComboBox()
        self.refresh_ports()
        btn_refresh = QPushButton("Atualizar")
        btn_refresh.setFixedWidth(100)
        btn_refresh.clicked.connect(self.refresh_ports)
        config_lay.addWidget(QLabel("Porta Serial:"))
        config_lay.addWidget(self.port_combo, 1)
        config_lay.addWidget(btn_refresh)
        config_group.setLayout(config_lay)
        main_layout.addWidget(config_group)

        # --- Comandos (Modo e Botão na mesma linha) ---
        action_group = QGroupBox("Comandos")
        action_lay = QHBoxLayout()
        
        self.spin_mode = QSpinBox()
        self.spin_mode.setRange(0,5)
        self.spin_mode.setValue(5)
        self.spin_mode.setMinimumWidth(60)
        
        self.btn_start = QPushButton("INICIAR TROCA DE MODO")
        self.btn_start.setFixedHeight(35)
        self.btn_start.clicked.connect(self.start_process)
        
        action_lay.addWidget(QLabel("Modo:"))
        action_lay.addWidget(self.spin_mode)
        action_lay.addSpacing(10)
        action_lay.addWidget(self.btn_start, 1)
        
        action_group.setLayout(action_lay)
        main_layout.addWidget(action_group)

        # --- Registro de Ações ---
        log_header = QHBoxLayout()
        log_header.addWidget(QLabel("Registro de Ações"))
        btn_clear = QPushButton("Limpar")
        btn_clear.setFixedWidth(80)
        btn_clear.setStyleSheet("background-color: rgba(52, 152, 219, 0.4); font-size: 9pt;")
        btn_clear.clicked.connect(lambda: self.text_log.clear())
        log_header.addStretch()
        log_header.addWidget(btn_clear)
        
        main_layout.addLayout(log_header)
        self.text_log = QTextEdit()
        self.text_log.setReadOnly(True)
        main_layout.addWidget(self.text_log)

    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for p in ports: self.port_combo.addItem(p.device)
        if not ports:
            self.port_combo.addItem("Nenhuma porta encontrada")
            self.port_combo.setEnabled(False)
        else: self.port_combo.setEnabled(True)

    def append_log(self, text, color="#DDDDDD"):
        html = f'<span style="color:{color};">• {text}</span>'
        self.text_log.append(html)
        self.text_log.moveCursor(QTextCursor.End)

    def start_process(self):
        port = self.port_combo.currentText()
        if "Nenhuma" in port: return
        
        self.btn_start.setEnabled(False)
        self.append_log(f"Iniciando rotina para Porta: {port} | Alvo: Modo {self.spin_mode.value()}", "#3498DB")
        
        self.worker = ComWorker(port, self.spin_mode.value())
        self.worker.log_signal.connect(self.append_log)
        self.worker.finished_signal.connect(self.process_finished)
        self.worker.start()

    def process_finished(self):
        self.btn_start.setEnabled(True)
        self.append_log("Pronto.", "#5DADE2")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = SerialApp()
    ex.show()
    sys.exit(app.exec_())