package com.careri.APLdrive;

import java.io.*;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.util.Properties;

public class Main {
    private static Double delta_t;
    private static Double angle_velocity;
    private static Double acc;
    private static String target_ip;
    private static int target_port;
    private static String receive_guide_ip;
    private static int receive_guide_port ;
    private static Double initial_longti;
    private static Double initial_lat;
    private static Double initial_height;
    private static Double initial_speed;
    private static Double initial_head;
    private static String receive_joystick_ip;
    private static int receive_joystick_port ;
    private static final double Earth_LongAxis = 6378137;
    private static final double Earth_ShortAxis = 6356752.3142;
    private static final double Ownpoint_lon = 121.81618;
    private static final double Ownpoint_lat = 31.160549;
    private static final double Ownpoint_h = 0;

    public static void getProperty() throws IOException {
        Properties prop  = new Properties();
        InputStream in = new BufferedInputStream(new FileInputStream("./conf/aircraft.properties"));
        prop.load(in);
        delta_t = Double.valueOf(prop.getProperty("delta_t"));
        angle_velocity = Double.valueOf(prop.getProperty("angle_velocity"));
        acc = Double.valueOf(prop.getProperty("acc"));
        target_ip = prop.getProperty("target_ip");
        receive_guide_ip = prop.getProperty("receive_guide_ip");
        target_port = Integer.parseInt(prop.getProperty("target_port"));
        receive_guide_port  = Integer.parseInt(prop.getProperty("receive_guide_port"));
        initial_longti = Double.valueOf(prop.getProperty("initial_longti"));
        initial_lat = Double.valueOf(prop.getProperty("initial_lat"));
        initial_height = Double.valueOf(prop.getProperty("initial_height"));
        initial_speed = Double.valueOf(prop.getProperty("initial_speed"));
        initial_head = Double.valueOf(prop.getProperty("initial_head"));
        receive_joystick_port  = Integer.parseInt(prop.getProperty("receive_joystick_port"));
        receive_joystick_ip  = prop.getProperty("receive_joystick_ip");
        in.close();
    }

    public static double bytes2Double(byte[] arr) {
        long value = 0;
        for (int i = 0; i < 8; i++) {
            value |= ((long) (arr[i] & 0xff)) << (8 * i);
        }
        return Double.longBitsToDouble(value);
    }

    public static Position WGS_to_ECEF(double lon, double lat, double h){
        Position p = new Position();
        double f = (Earth_LongAxis - Earth_ShortAxis) / Earth_LongAxis;
        double e_sq = f * (2 - f);
        double lamb = lat / 180 * Math.PI;
        double phi = lon / 180 * Math.PI;
        double s = Math.sin(lamb);
        double N = Earth_LongAxis / Math.sqrt(1 - e_sq * s * s);
        double sin_lambda = Math.sin(lamb);
        double cos_lambda = Math.cos(lamb);
        double sin_phi = Math.sin(phi);
        double cos_phi = Math.cos(phi);
        p.x = (h + N) * cos_lambda * cos_phi;
        p.y = (h + N) * cos_lambda * sin_phi;
        p.z = (h + (1 - e_sq) * N) * sin_lambda;
        return p;
    }

    public static Position ECEF_to_ENU(double x, double y, double z, double n_lon, double n_lat, double n_h){
        Position p = new Position();
        double lamb = n_lat / 180 * Math.PI;
        double phi = n_lon / 180 * Math.PI;
        double s = Math.sin(lamb);
        double f = (Earth_LongAxis - Earth_ShortAxis) / Earth_LongAxis;
        double e_sq = f * (2 - f);
        double N = Earth_LongAxis / Math.sqrt(1 - e_sq * s * s);

        double sin_lambda = Math.sin(lamb);
        double cos_lambda = Math.cos(lamb);
        double sin_phi = Math.sin(phi);
        double cos_phi = Math.cos(phi);

        double x0 = (n_h + N) * cos_lambda * cos_phi;
        double y0 = (n_h + N) * cos_lambda * sin_phi;
        double z0 = (n_h + (1 - e_sq) * N) * sin_lambda;

        double xd = x - x0;
        double yd = y - y0;
        double zd = z - z0;

        p.x = -sin_phi * xd + cos_phi * yd;
        p.y = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
        p.z = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

        return p;
    }

    public static Position WGS_to_ENU(double x, double y, double z, double n_lon, double n_lat, double n_h){
        Position p1 = new Position();
        Position p2 = new Position();
        p1 =  WGS_to_ECEF(x, y, z);
        p2 = ECEF_to_ENU(p1.x, p1.y, p1.z, n_lon, n_lat, n_h);
        return p2;
    }

    public static Position ENU_to_ECEF(double x, double y, double z, double n_lon, double n_lat, double n_h){
        Position p = new Position();
        double lamb = n_lat / 180 * Math.PI;
        double phi = n_lon / 180 * Math.PI;
        double s = Math.sin(lamb);
        double f = (Earth_LongAxis - Earth_ShortAxis) / Earth_LongAxis;
        double e_sq = f * (2 - f);
        double N = Earth_LongAxis / Math.sqrt(1 - e_sq * s * s);
        double sin_lambda = Math.sin(lamb);
        double cos_lambda = Math.cos(lamb);
        double sin_phi = Math.sin(phi);
        double cos_phi = Math.cos(phi);
        double x0 = (n_h + N) * cos_lambda * cos_phi;
        double y0 = (n_h + N) * cos_lambda * sin_phi;
        double z0 = (n_h + (1 - e_sq) * N) * sin_lambda;

        double xd = -sin_lambda * cos_phi * y - sin_phi * x + cos_lambda * cos_phi * z;
        double yd = -sin_lambda * sin_phi * y + cos_phi * x + cos_lambda * sin_phi * z;
        double zd = cos_lambda * y + sin_lambda * z;
        p.x = xd + x0;
        p.y = yd + y0;
        p.z = zd + z0;
        return p;
    }

    public static Postion_WGS ECEF_to_WGS(double x, double y, double z){
        Postion_WGS p =new Postion_WGS();
        double B_;
        double N;
        double dSemiMajorAxis = 6378137.0;//椭球长半轴
        double e2 = 0.0066943799013;//第一偏心率的平方
        double R = Math.sqrt(x * x + y * y);
        double B0 = Math.atan2(z, R);
        double tick = 0;
        while (true)
        {
            N = dSemiMajorAxis / Math.sqrt(1.0 - e2 * Math.sin(B0) * Math.sin(B0));
            B_ = Math.atan2(z + N * e2 * Math.sin(B0), R);

            if (Math.abs(B_ - B0) < 1.0e-10)
            {
                break;
            }

            B0 = B_;
            tick += 1;
            if (tick >= 10000)
            {
                break;
            }
        }
        double L_ = Math.atan2(y, x);
        p.h = R / Math.cos(B_) - N;
        //弧度转换成经纬度
        p.lat = B_ * 180 / Math.PI;
        p.lon = L_ * 180 / Math.PI;
        return p;
    }

    public static Postion_WGS ENU_to_WGS(double x, double y, double z, double n_lon, double n_lat, double n_h){
        Position p1 = new Position();
        Postion_WGS p2 = new Postion_WGS();
        p1 = ENU_to_ECEF(x, y, z, n_lon, n_lat, n_h);
        p2 = ECEF_to_WGS(p1.x, p1.y, p1.z);
        return p2;
    }

    public static byte[] double2Bytes(double d) {
        long value = Double.doubleToRawLongBits(d);
        byte[] byteRet = new byte[8];
        for (int i = 0; i < 8; i++) {
            byteRet[i] = (byte) ((value >> 8 * i) & 0xff);
        }
        return byteRet;
    }

    public static byte[] concat(byte[] a, byte[] b, byte[] c, byte[] d, byte[] e) {
        byte[] t= new byte[a.length+b.length+c.length+d.length+e.length];
        System.arraycopy(a, 0, t, 0, a.length);
        System.arraycopy(b, 0, t, a.length, b.length);
        System.arraycopy(c, 0, t, a.length+b.length, c.length);
        System.arraycopy(d, 0, t, a.length+b.length+c.length, d.length);
        System.arraycopy(e, 0, t, a.length+b.length+c.length+d.length, e.length);
        return t;
    }

    public static void sendPOSI(APLposition new_POSI){
        byte[] a = double2Bytes(new_POSI.longti);
        byte[] b = double2Bytes(new_POSI.lat);
        byte[] c = double2Bytes(new_POSI.height);
        byte[] d = double2Bytes(new_POSI.speed);
        byte[] e = double2Bytes(new_POSI.heading);
        byte[] buff = concat(a,b,c,d,e);
        DatagramPacket outPacket = null;
        try{
            DatagramSocket socket = new DatagramSocket();
            outPacket = new DatagramPacket(new byte[0], 0, InetAddress.getByName(target_ip), target_port);
            outPacket.setData(buff);
            socket.send(outPacket);
            socket.close();
        } catch (SocketException socketException) {
            socketException.printStackTrace();
        } catch (IOException ioException) {
            ioException.printStackTrace();
        }
    }

    public static double turnCalc(double cur_heading, double target_heading, double turning_rate) {
        if(Math.abs(cur_heading - target_heading) <= 180){
            if (cur_heading > target_heading) {
                cur_heading = cur_heading - turning_rate;
                if (cur_heading < target_heading) {
                    cur_heading = target_heading;
                }
            }
            else{
                cur_heading = cur_heading + turning_rate;
                if (cur_heading > target_heading) {
                    cur_heading = target_heading;
                }
            }
        }
        else{
            // 判断在当前航向的左侧还是右侧
            if (cur_heading <= 180) {  // 向0逆时针旋转
                // 判断是否转到0了
                if (cur_heading <= turning_rate) {
                    cur_heading  = 360 - cur_heading + turning_rate;
                }
                else {
                    cur_heading = cur_heading - turning_rate;
                }
            }
            else {  // 向0顺时针旋转
                if (cur_heading >= 360 - turning_rate) {
                    cur_heading = cur_heading + turning_rate -360;
                }
                else {
                    cur_heading = cur_heading + turning_rate;
                }
            }
        }
        return cur_heading;
    }


    public static APLposition getPOSI(APLposition cur_POSI, GuidSignal gs, Joystick js){
        /*
        * 1. 先判断是否收到杆信号，0.02以内认为无信号
        * 2. 根据杆信号，得到当前加速度和转向率,有杆信号就优先按照杆信号运动
        * 3. 根据指引信号，无信号则按当前状态，有信号则先判断当前是否达到指引状态，然后再判断是否进行转向和加减速
        * 4. 计算位置
        * 5. UDP发送
        * 6. 睡眠20ms
        * */
        double temp_acc;
        double temp_anglecc;
        Position temp_cur_POSI;
        Position temp_new_POSI = new Position();
        Postion_WGS temp_POSI_WGS = new Postion_WGS();
        APLposition new_POSI = new APLposition();
        if ((Math.abs(js.acc_And_dcc) > 0.02) && (Math.abs(js.angle_offset) > 0.02)){  // 有杆信号
            temp_acc = js.acc_And_dcc * acc;
            temp_anglecc = js.angle_offset * angle_velocity;
            new_POSI.speed = cur_POSI.speed + temp_acc * delta_t;
            new_POSI.heading = cur_POSI.heading + temp_anglecc * delta_t;
            new_POSI.height = cur_POSI.height;
            temp_cur_POSI = WGS_to_ENU(cur_POSI.longti, cur_POSI.lat, cur_POSI.height, Ownpoint_lon, Ownpoint_lat, Ownpoint_h);
            temp_new_POSI.x = temp_cur_POSI.x + cur_POSI.speed * delta_t * Math.sin(cur_POSI.heading);
            temp_new_POSI.y = temp_cur_POSI.y + cur_POSI.speed * delta_t * Math.cos(cur_POSI.heading);
            temp_new_POSI.z = temp_cur_POSI.z;
            temp_POSI_WGS = ENU_to_WGS(temp_new_POSI.x, temp_new_POSI.y, temp_new_POSI.z, Ownpoint_lon, Ownpoint_lat, Ownpoint_h);
            new_POSI.lat = temp_POSI_WGS.lat;
            new_POSI.longti = temp_POSI_WGS.lon;
        }
        else{  // 无杆信号
            if ((gs.target_angle >= 0) && (gs.target_speed >= 0)){ // 有指引信号，先判断是否达到指引状态
                temp_acc = acc;
                temp_anglecc = angle_velocity;
                if ((Math.abs(cur_POSI.heading - gs.target_angle) < (delta_t * angle_velocity))  // 达到指引状态
                        && (Math.abs(cur_POSI.speed - gs.target_speed) < (delta_t * acc))){
                    new_POSI.speed = cur_POSI.speed;
                    new_POSI.heading = cur_POSI.heading;
                }
                else{ // 尚未达到指引状态
                    new_POSI.speed = cur_POSI.speed + temp_acc * delta_t;
                    new_POSI.heading = turnCalc(cur_POSI.heading, gs.target_angle, temp_anglecc * delta_t);
                }
                new_POSI.height = cur_POSI.height;
                temp_cur_POSI = WGS_to_ENU(cur_POSI.longti, cur_POSI.lat, cur_POSI.height, Ownpoint_lon, Ownpoint_lat, Ownpoint_h);
                temp_new_POSI.x = temp_cur_POSI.x + cur_POSI.speed * delta_t * Math.sin(cur_POSI.heading);
                temp_new_POSI.y = temp_cur_POSI.y + cur_POSI.speed * delta_t * Math.cos(cur_POSI.heading);
                temp_new_POSI.z = temp_cur_POSI.z;
                temp_POSI_WGS = ENU_to_WGS(temp_new_POSI.x, temp_new_POSI.y, temp_new_POSI.z, Ownpoint_lon, Ownpoint_lat, Ownpoint_h);
                new_POSI.lat = temp_POSI_WGS.lat;
                new_POSI.longti = temp_POSI_WGS.lon;
            }
            else{  // 没有指引信号
                temp_acc = 0;
                temp_anglecc = 0;
                new_POSI.speed = cur_POSI.speed;
                new_POSI.heading = cur_POSI.heading;
                new_POSI.height = cur_POSI.height;
                if (cur_POSI.speed == 0){
                    new_POSI.lat = cur_POSI.lat;
                    new_POSI.longti = cur_POSI.longti;
                }
                else{
                    temp_cur_POSI = WGS_to_ENU(cur_POSI.longti, cur_POSI.lat, cur_POSI.height, Ownpoint_lon, Ownpoint_lat, Ownpoint_h);
                    temp_new_POSI.x = temp_cur_POSI.x + cur_POSI.speed * delta_t * Math.sin(cur_POSI.heading);
                    temp_new_POSI.y = temp_cur_POSI.y + cur_POSI.speed * delta_t * Math.cos(cur_POSI.heading);
                    temp_new_POSI.z = temp_cur_POSI.z;
                    temp_POSI_WGS = ENU_to_WGS(temp_new_POSI.x, temp_new_POSI.y, temp_new_POSI.z, Ownpoint_lon, Ownpoint_lat, Ownpoint_h);
                    new_POSI.lat = temp_POSI_WGS.lat;
                    new_POSI.longti = temp_POSI_WGS.lon;
                }
            }
        }
        System.out.println("acc:" + temp_acc + "; tcc:" + temp_anglecc);
        return new_POSI;
    }

    static class ReceiveGuidThread implements Runnable{
        private DatagramSocket socket;
        GuidSignal guidsignal = new GuidSignal();

        @Override
        public void run() {  // 收到2个double：指引速度、指引航向
            try{
                socket = new DatagramSocket(receive_guide_port);
                while (true){
                    byte[] buf = new byte[1024];
                    DatagramPacket recv_msg = new DatagramPacket(buf, buf.length);
                    socket.receive(recv_msg);
                    if(recv_msg.getAddress().getHostAddress().equals(receive_guide_ip)){
                        byte[] datas = recv_msg.getData();
                        byte[] temp_data_1 = new byte[8];
                        System.arraycopy(datas, 0, temp_data_1, 0,8);
                        double data_1 = bytes2Double(temp_data_1);
                        guidsignal.setTarget_speed(data_1);
                        byte[] temp_data_2 = new byte[8];
                        System.arraycopy(datas, 8, temp_data_2, 0,8);
                        double data_2 = bytes2Double(temp_data_2);
                        guidsignal.setTarget_angle(data_2);
                        System.out.println("Received guid signal: " + guidsignal.target_speed + " " + guidsignal.target_angle);
                    }
                }
            } catch (SocketException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }finally {
                if (socket != null){
                    socket.close();
                    System.out.println("Recive Guid socket close");
                }
            }
        }
    }

    static class ReceiveJoystickThread implements Runnable{
        private DatagramSocket socket;
        Joystick joysticksignal = new Joystick();

        @Override
        public void run() {  // 收到2个double：加速度系数、转向率系数
            try{
                socket = new DatagramSocket(receive_joystick_port);
                while (true){
                    byte[] buf = new byte[1024];
                    DatagramPacket recv_msg = new DatagramPacket(buf, buf.length);
                    socket.receive(recv_msg);
                    if(recv_msg.getAddress().getHostAddress().equals(receive_joystick_ip)){
                        byte[] datas = recv_msg.getData();
                        byte[] temp_data_1 = new byte[8];
                        System.arraycopy(datas, 0, temp_data_1, 0,8);
                        double data_1 = bytes2Double(temp_data_1);
                        joysticksignal.setAcc_And_dcc(data_1);
                        byte[] temp_data_2 = new byte[8];
                        System.arraycopy(datas, 8, temp_data_2, 0,8);
                        double data_2 = bytes2Double(temp_data_2);
                        joysticksignal.setAngle_offset(data_2);
                        System.out.println("Received joystick signal: " + joysticksignal.acc_And_dcc + " " + joysticksignal.angle_offset);
                    }
                }
            } catch (SocketException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }finally {
                if (socket != null){
                    socket.close();
                    System.out.println("Recive Joystick socket close");
                }
            }
        }
    }

    public static void main(String[] args) throws IOException, InterruptedException {
        getProperty();
        ReceiveGuidThread rgt = new ReceiveGuidThread();
        ReceiveJoystickThread rjt = new ReceiveJoystickThread();

        Thread t1 = new Thread(rgt);
        Thread t2 = new Thread(rjt);

        t1.start();
        t2.start();

        // 初始化位置
        APLposition cur_posi = new APLposition();
        APLposition new_posi = new APLposition();
        cur_posi.longti = initial_longti;
        cur_posi.lat = initial_lat;
        cur_posi.height = initial_height;
        cur_posi.speed = initial_speed;
        cur_posi.heading = initial_head;
        new_posi.lat = cur_posi.lat;
        new_posi.longti = cur_posi.longti;
        new_posi.height = cur_posi.height;
        new_posi.speed = cur_posi.speed;
        new_posi.heading = cur_posi.heading;
        while (true){
            new_posi = getPOSI(new_posi, rgt.guidsignal, rjt.joysticksignal);
            Thread.sleep(delta_t.longValue() * 1000);
            System.out.println(new_posi.longti + " " + new_posi.lat + " " +new_posi.height + " " +
                    new_posi.speed + " " + new_posi.heading + " ");
            sendPOSI(new_posi);
        }
    }
}
