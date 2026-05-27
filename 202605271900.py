"""
步进电机上位机
pip install pyserial
"""
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time


class App:
    def __init__(self):
        self.ser = None
        self.connected = False
        self.root = tk.Tk()
        self.root.title("步进电机控制 v1.0")
        self.root.geometry("850x650")
        self.build_ui()
        self.refresh_ports()
    
    def build_ui(self):
        # 串口区
        f1 = ttk.LabelFrame(self.root, text="串口连接", padding=5)
        f1.pack(fill=tk.X, padx=5, pady=3)
        
        ttk.Label(f1, text="串口:").pack(side=tk.LEFT, padx=3)
        self.cb_port = ttk.Combobox(f1, width=10, state="readonly")
        self.cb_port.pack(side=tk.LEFT, padx=3)
        
        ttk.Label(f1, text="波特率:").pack(side=tk.LEFT, padx=3)
        self.cb_baud = ttk.Combobox(f1, width=8, values=["9600","19200","38400","57600","115200"])
        self.cb_baud.set("115200")
        self.cb_baud.pack(side=tk.LEFT, padx=3)
        
        ttk.Button(f1, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT, padx=3)
        self.btn_conn = ttk.Button(f1, text="连接", command=self.toggle_conn)
        self.btn_conn.pack(side=tk.LEFT, padx=3)
        self.lbl_st = ttk.Label(f1, text="●未连接", foreground="red")
        self.lbl_st.pack(side=tk.LEFT, padx=8)
        
        # 主体
        f_main = ttk.Frame(self.root)
        f_main.pack(fill=tk.BOTH, expand=True, padx=5, pady=3)
        
        # 左：参数+控制
        f_left = ttk.Frame(f_main)
        f_left.pack(side=tk.LEFT, fill=tk.Y, padx=(0,5))
        
        # 参数设置
        fp = ttk.LabelFrame(f_left, text="⚙ 参数设置", padding=5)
        fp.pack(fill=tk.X, pady=3)
        self.v_ppr   = self._ent(fp, 0, "每圈脉冲数:", "200")
        self.v_ppm   = self._ent(fp, 1, "每mm脉冲数:", "200")
        self.v_maxf  = self._ent(fp, 2, "最高频率Hz:", "25000")
        self.v_accel = self._ent(fp, 3, "加速步数:",   "200")
        self.v_decel = self._ent(fp, 4, "减速步数:",   "200")
        ttk.Button(fp, text="📥 应用参数", command=self.apply_params).grid(row=5, column=0, columnspan=3, pady=5)
        
        # 运动控制
        fm = ttk.LabelFrame(f_left, text="🎯 运动控制", padding=5)
        fm.pack(fill=tk.X, pady=3)
        
        ttk.Label(fm, text="方式:").grid(row=0, column=0, sticky=tk.W)
        self.cb_mode = ttk.Combobox(fm, width=8, values=["步数","毫米","圈数"], state="readonly")
        self.cb_mode.set("步数")
        self.cb_mode.grid(row=0, column=1)
        
        self.v_val = self._ent(fm, 1, "运动量:", "1000")
        
        ttk.Label(fm, text="方向:").grid(row=2, column=0, sticky=tk.W)
        self.cb_dir = ttk.Combobox(fm, width=8, values=["正转","反转"], state="readonly")
        self.cb_dir.set("正转")
        self.cb_dir.grid(row=2, column=1)
        
        self.v_ret = self._ent(fm, 3, "往返次数:", "0")
        
        bf = ttk.Frame(fm)
        bf.grid(row=4, column=0, columnspan=3, pady=5)
        ttk.Button(bf, text="▶ 开始", command=self.start).pack(side=tk.LEFT, padx=5)
        ttk.Button(bf, text="■ 停止", command=self.stop).pack(side=tk.LEFT, padx=5)
        tk.Button(bf, text="🔴 急停", bg="red", fg="white", command=self.estop).pack(side=tk.LEFT, padx=5)
        ttk.Button(bf, text="🔄 复位", command=self.reset).pack(side=tk.LEFT, padx=5)
        
        # 右：状态+日志
        f_right = ttk.Frame(f_main)
        f_right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        fs = ttk.LabelFrame(f_right, text="📊 状态", padding=5)
        fs.pack(fill=tk.X, pady=3)
        self.lbl_pos = ttk.Label(fs, text="位置: 0步", font=("Arial",14,"bold"), foreground="blue")
        self.lbl_pos.pack(anchor=tk.W)
        self.lbl_state = ttk.Label(fs, text="状态: IDLE", font=("Arial",14,"bold"), foreground="green")
        self.lbl_state.pack(anchor=tk.W)
        self.lbl_ret = ttk.Label(fs, text="往返: 0/0", font=("Arial",12), foreground="purple")
        self.lbl_ret.pack(anchor=tk.W)
        
        fl = ttk.LabelFrame(f_right, text="📝 日志", padding=3)
        fl.pack(fill=tk.BOTH, expand=True)
        self.txt = scrolledtext.ScrolledText(fl, height=15, font=("Consolas",9))
        self.txt.pack(fill=tk.BOTH, expand=True)
        ttk.Button(fl, text="清空", command=lambda: self.txt.delete(1.0, tk.END)).pack(pady=2)
        
        # 快捷按钮
        fb = ttk.LabelFrame(self.root, text="⚡ 快捷操作", padding=3)
        fb.pack(fill=tk.X, padx=5, pady=3)
        ttk.Button(fb, text="查询状态", command=lambda: self.send("$STATUS")).pack(side=tk.LEFT, padx=3)
        ttk.Button(fb, text="正转100步", command=lambda: self.send("$MOVE,STEPS,100,0")).pack(side=tk.LEFT, padx=3)
        ttk.Button(fb, text="反转100步", command=lambda: self.send("$MOVE,STEPS,100,1")).pack(side=tk.LEFT, padx=3)
        ttk.Button(fb, text="正转1圈", command=lambda: self.send("$MOVE,REV,1,0")).pack(side=tk.LEFT, padx=3)
        ttk.Button(fb, text="反转1圈", command=lambda: self.send("$MOVE,REV,1,1")).pack(side=tk.LEFT, padx=3)
    
    def _ent(self, p, r, label, default):
        ttk.Label(p, text=label).grid(row=r, column=0, sticky=tk.W, pady=2)
        v = ttk.Entry(p, width=10)
        v.insert(0, default)
        v.grid(row=r, column=1, pady=2)
        return v
    
    def refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.cb_port['values'] = ports
        if ports: self.cb_port.set(ports[0])
    
    def toggle_conn(self):
        if self.connected:
            self.connected = False
            if self.ser: self.ser.close()
            self.btn_conn.configure(text="连接")
            self.lbl_st.configure(text="●未连接", foreground="red")
        else:
            port = self.cb_port.get()
            if not port: return
            try:
                self.ser = serial.Serial(port, int(self.cb_baud.get()), timeout=1)
                self.connected = True
                self.btn_conn.configure(text="断开")
                self.lbl_st.configure(text="●已连接", foreground="green")
                threading.Thread(target=self.rx_loop, daemon=True).start()
            except Exception as e:
                messagebox.showerror("错误", str(e))
    
    def rx_loop(self):
        while self.connected:
            try:
                if self.ser and self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line: self.root.after(0, self.on_rx, line)
                time.sleep(0.01)
            except: break
    
    def on_rx(self, data):
        self.txt.insert(tk.END, f"RX: {data}\n")
        self.txt.see(tk.END)
        
        if "STATUS" in data:
            for p in data.split(","):
                if p.startswith("POS="):
                    self.lbl_pos.configure(text=f"位置: {p[4:]}步")
                elif p.startswith("STATE="):
                    s = p[6:]
                    c = {"IDLE":"green","ACCEL":"orange","CRUISE":"blue",
                         "DECEL":"purple","DONE":"green","ESTOP":"red"}.get(s,"black")
                    self.lbl_state.configure(text=f"状态: {s}", foreground=c)
                elif p.startswith("RET="):
                    self.lbl_ret.configure(text=f"往返: {p[4:]}")
        
        if "RETURN" in data:
            self.send("$STATUS")
    
    def send(self, cmd):
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        self.ser.write((cmd + "\r\n").encode())
        self.txt.insert(tk.END, f"TX: {cmd}\n")
        self.txt.see(tk.END)
    
    def apply_params(self):
        try:
            self.send(f"$SET,PPR,{self.v_ppr.get()}")
            time.sleep(0.05)
            self.send(f"$SET,PPM,{self.v_ppm.get()}")
            time.sleep(0.05)
            self.send(f"$SET,MAXFREQ,{self.v_maxf.get()}")
            time.sleep(0.05)
            self.send(f"$SET,ACCEL,{self.v_accel.get()}")
            time.sleep(0.05)
            self.send(f"$SET,DECEL,{self.v_decel.get()}")
        except: pass
    
    def start(self):
        try: val = int(self.v_val.get())
        except: messagebox.showerror("错误", "请输入数字"); return
        d = 0 if self.cb_dir.get() == "正转" else 1
        ret = int(self.v_ret.get())
        if ret > 0:
            self.send(f"$RETURN,{ret}")
            time.sleep(0.05)
        m = {"步数":"STEPS", "毫米":"MM", "圈数":"REV"}[self.cb_mode.get()]
        self.send(f"$MOVE,{m},{val},{d}")
    
    def stop(self):  self.send("$STOP")
    def estop(self): self.send("$ESTOP")
    def reset(self): self.send("$RESET")
    
    def run(self): self.root.mainloop()


if __name__ == "__main__":
    App().run()