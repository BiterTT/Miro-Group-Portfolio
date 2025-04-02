    


    def flash_color_real(self, color = "red", duration=3):
        if color == "red":
            rgb = (255,0,0)
        elif color == "green":
            rgb =(0,255,0)
        else:
            rgb = (255,255,255)
        
        r,g,b = rgb
        color_val=(r<<16)|(g<<8)|b|0xFF000000

        for i in range(6):
            self.msg_illum.data[i] = color_val
        self.pub_illum.publish(self.msg_illum)

        rospy.sleep(duration)

        for i in range(6):
            self.msg_illum.data[i] = 0xFF000000
        self.pub_illum.publish(self.msg_illum)



    def illum_Shine(self, xcc, shine_flag):
        if shine_flag == True:
            q = int(xcc * -127 + 128)
            for i in range(0, 3):
                self.msg_illum.data[i] = (q << ((2-i) * 8)) | 0xFF000000
            for i in range(3, 6): 
                self.msg_illum.data[i] = (q << ((i-3) * 8)) | 0xFF000000
        else:
            # 不闪烁时设为全黑（只有 alpha 通道）
            for i in range(6):
                self.msg_illum.data[i] = 0xFF000000
        self.pub_illum.publish(self.msg_illum)