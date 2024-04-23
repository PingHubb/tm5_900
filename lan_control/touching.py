import tkinter as tk
from tkinter import filedialog
import polyscope as ps
import trimesh
import numpy as np
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.animation import FuncAnimation
import multiprocessing as mp
import CH341N

np.set_printoptions(threshold=10000, linewidth=10000) #全部输出
channelDrive = 13
channelSensor = 9
channelDriveS = 0
channelSensorS = 1

target_points = np.array([[-6.6,11.3,0.5],[-6.9,7.4,3.0],[-6.3,5,3.5],[-5.9,3.3,3.9],[-5.6,0.7,2.7],[-5.3,-1.2,1.8],[-5.2,-3.3,1.2],[-5.4,-5,1.4],[-5.9,-6.6,2.2],[-6.2,-8.2,2.9],[-6.3,-9.9,3.4]
                            ,[-4.7,12,0.7],[-5.2,10.8,1.9],[-5.3,9.1,3.1],[-5.3,6.8,4.2],[-5.2,4.9,5.2],[-5.1,3.2,5.6],[-5,0.8,4.6],[-4.7,-1.4,3.5],[-4.5,-3,2.9],[-4.7,-4.8,3],[-4.9,-6.5,3.7],[-5.1,-8.2,4.4],[-5.2,-9.9,4.7]
                            ,[-3.4,11.6,2],[-3.6,10.1,3.1],[-3.6,8.6,4],[-3.6,6.7,5.1],[-3.6,4.8,6.1],[-3.6,2.9,6.4],[-3.6,0.8,5.8],[-3.5,-1.2,5],[-3.4,-2.9,4.4],[-3.4,-4.7,4.3],[-3.5,-6.4,4.8],[-3.6,-8.1,5.3],[-3.6,-9.9,5.6]
                            ,[-1.8,11.1,3.1],[-1.8,9.5,3.9],[-1.8,7.9,4.8],[-1.8,6.3,5.5],[-1.8,4.6,6.2],[-1.8,2.5,6.4],[-1.8,0.7,6.1],[-1.8,-1.1,5.6],[-1.8,-2.8,5.2],[-1.8,-4.6,5.1],[-1.8,-6.3,5.5],[-1.8,-8.1,5.8],[-1.8,-9.9,6]
                            ,[0,10.9,3.5],[0,9.3,4.3],[0,7.7,5],[0,6,5.7],[0,4.3,6.2],[0,2.5,6.3],[0,0.7,6.1],[0,-1,5.8],[0,-2.8,5.4],[0,-4.6,5.4],[0,-6.4,5.8],[0,-8.1,6],[0,-9.9,6.2]
                            ,[1.8,11.1,3.1],[1.8,9.5,3.9],[1.8,7.9,4.8],[1.8,6.3,5.5],[1.8,4.6,6.2],[1.8,2.5,6.4],[1.8,0.7,6.1],[1.8,-1.1,5.6],[1.8,-2.8,5.2],[1.8,-4.6,5.1],[1.8,-6.3,5.5],[1.8,-8.1,5.8],[1.8,-9.9,6]
                            ,[3.4,11.6,2],[3.6,10.1,3.1],[3.6,8.6,4],[3.6,6.7,5.1],[3.6,4.8,6.1],[3.6,2.9,6.4],[3.6,0.8,5.8],[3.5,-1.2,5],[3.4,-2.9,4.4],[3.4,-4.7,4.3],[3.5,-6.4,4.8],[3.6,-8.1,5.3],[3.6,-9.9,5.6]
                            ,[4.7,12,0.7],[5.2,10.8,1.9],[5.3,9.1,3.1],[5.3,6.8,4.2],[5.2,4.9,5.2],[5.1,3.2,5.6],[5,0.8,4.6],[4.7,-1.4,3.5],[4.5,-3,2.9],[4.7,-4.8,3],[4.9,-6.5,3.7],[5.1,-8.2,4.4],[5.2,-9.9,4.7]
                            ,[6.6,11.3,0.5],[6.9,7.4,3.0],[6.3,5,3.5],[5.9,3.3,3.9],[5.6,0.7,2.7],[5.3,-1.2,1.8],[5.2,-3.3,1.2],[5.4,-5,1.4],[5.9,-6.6,2.2],[6.2,-8.2,2.9],[6.3,-9.9,3.4]])  # dim: 113*3
target_points = np.hstack((target_points, np.zeros((113, 1))))  # dim: 113*4
ignore_points = np.array([[0,0],[0,2],[8,0],[8,2]])

def polyscope_function(shared_array):
     # 创建一个Tkinter窗口
    root = tk.Tk()
    root.withdraw()  # 隐藏窗口
    
    file_path = filedialog.askopenfilename(title="Select the obj.file")

    def redraw():
        data = np.frombuffer(shared_array, dtype=np.int32).reshape((channelDrive, channelSensor))
        dataT = data.T
        x = 0
        for i, line in enumerate(dataT):
            for j, p in enumerate(dataT[i]):
                if any(np.array_equal(x, [i,j]) for x in ignore_points):
                    continue
                target_points[x][3] = dataT[i][j]
                x = x + 1
        for rings in targetVertexsRings:
            for index, ring in enumerate(rings):
                for v in ring:
                    # colors[v] = (1,0,0)
                    colors[v] = (1,1-float(target_points[index][3])/155,1-float(target_points[index][3])/155)
        # colors = np.random.rand(len(vertices),3)
        mesh.add_color_quantity("my color", colors, defined_on='vertices', enabled=True)
        

        return
    
    def findTargetRings(count,adjacency_graph, targetVertexsRings):
        for i in range(3):
            targetVertexsRing = []
            for index,ringVertexs in enumerate(targetVertexsRings[i]):
                vsRing = set()
                for ringVertex in ringVertexs:
                    vsRing.update(adjacency_graph[ringVertex])
                vsRing_list = list(vsRing)
                vsRing_list = [x for x in vsRing_list if x not in targetVertexsRings[i][index]]
                if(i>0):
                    vsRing_list = [x for x in vsRing_list if x not in targetVertexsRings[i-1][index]]
                targetVertexsRing.append(vsRing_list)
            targetVertexsRings.append(targetVertexsRing)
        return targetVertexsRings

    def findTargetVertex(targets,vertexs,obj):
        adjacency_graph = obj.vertex_adjacency_graph
        targetVertexsRings = []
        closest_vertex_idxs = []
        for target in targets:
            distances = []
            for vertex in vertexs:
                distances.append(np.linalg.norm(vertex - target[:3]))
            closest_vertex_idx = []
            closest_vertex_idx.append(np.argmin(distances))
            closest_vertex_idxs.append(closest_vertex_idx)
        targetVertexsRings.append(closest_vertex_idxs)
        findTargetRings(5,adjacency_graph,targetVertexsRings)
        return targetVertexsRings

    with open(file_path, 'r') as f:
        global faces, obj, vertices, mesh, colors,targetVertexsRings
        obj = trimesh.load_mesh(file_path)
        vertices = obj.vertices
        faces = obj.faces
        colors = np.ones((len(vertices),3),np.float64)
        targetVertexsRings = findTargetVertex(target_points,vertices,obj)
        
        # 初始化Polyscope
        ps.init()
        ps.set_user_callback(redraw)
        ps.set_max_fps(-1)
        # 创建一个Mesh对象并添加网格数据
        mesh = ps.register_surface_mesh("My Mesh", vertices, faces)
        ps.show()

def heatmap_function(shared_array):

    data = np.frombuffer(shared_array, dtype=np.int32).reshape((channelDrive, channelSensor))

    # 自定义颜色映射
    colors = [(1, 1, 1), (1, 0, 0)]  # 从纯白到纯红的渐变
    cmap = LinearSegmentedColormap.from_list('CustomMap', colors)


    # 创建热力图
    fig, ax = plt.subplots(figsize=(7,6*channelDrive/channelSensor))
    heatmap = ax.imshow(data, cmap=cmap,vmin=0, vmax=200)
    cbar = fig.colorbar(heatmap)

    # 添加数据标签
    texts = []

    # 更新函数，用于更新数据
    def update(frames,texts):
        for t in texts:
            t.remove()
        texts = []
        
        for i in range(channelDrive):
            for j in range(channelSensor):
                text = ax.text(j, i, format(data[i, j], ".0f"), ha="center", va="center", color='white' if data[i][j] > 100 else 'black')
                texts.append(text)
        heatmap.set_array(data)
        return [heatmap] + texts



    # 移除坐标轴
    ax.axis('off')
    # 调整子图边距，使其填满整个窗口
    plt.subplots_adjust(left=0.05, right=1, top=0.97, bottom=0.03)

    # 创建动画
    ani = FuncAnimation(fig, update, frames=None, interval=40, blit=True,fargs=(texts,))
    # 捕获 KeyboardInterrupt 异常来判断用户是否关闭窗口
    try:
        plt.show()
    except KeyboardInterrupt:
        pass

def camera_function():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
    cv2.namedWindow("capture", cv2.WINDOW_NORMAL)  # 创建可调整尺寸的窗口
    cv2.resizeWindow("capture", 858, 480)
    while(1):
        # get a frame
        ret, frame = cap.read()
        # show a frame
        cv2.imshow("capture", frame)
        key = cv2.waitKey(1)
        if cv2.getWindowProperty("capture", cv2.WND_PROP_VISIBLE) < 1:
            break
    cap.release()
    cv2.destroyAllWindows()

def data_function(shared_array):
    myUSB = CH341N.USBI2C(0x00,0x28)
    data = np.frombuffer(shared_array, dtype=np.int32).reshape((channelDrive, channelSensor))
    
    def channelCheck():
        touchSensorCount =0
        touchDriveCount =0
        while touchDriveCount == 0 or touchSensorCount == 0:
            touchSensorCount = int(myUSB.read(0x80,0x64)/16) +int(myUSB.read(0x80,0x64)%16)
            touchDriveCount = myUSB.read(0x80,0x62)%32 + myUSB.read(0x80,0x63)%32
        print("Channels Ready: Sensor = %d, Driver = %d" %(touchSensorCount,touchDriveCount))
        return touchDriveCount, touchSensorCount
        return 13, 9

    def getData():
        touchCaliData = np.zeros([touchDriveCount,touchSensorCount],dtype=int)
        touchRawData = np.zeros([touchDriveCount,touchSensorCount],dtype=int)
        touchDiffData = np.zeros([touchDriveCount,touchSensorCount],dtype=int)
        a = myUSB.readlong(0x81,0xC0,2*touchSensorCount*touchDriveCount)
        for i in range(touchDriveCount):
            for j in range(touchSensorCount):
                if a[2*(i*touchSensorCount+j)]*256 + a[2*(i*touchSensorCount+j)+1] != 0:
                    touchCaliData[i][j] = a[2*(i*touchSensorCount+j)]*256 + a[2*(i*touchSensorCount+j)+1]
        a = myUSB.readlong(0x8B,0x98,2*touchSensorCount*touchDriveCount)
        for i in range(touchDriveCount):
            for j in range(touchSensorCount):
                if a[2*(i*touchSensorCount+j)]*256 + a[2*(i*touchSensorCount+j)+1] != 0:
                    touchRawData[i][j] = a[2*(i*touchSensorCount+j)]*256 + a[2*(i*touchSensorCount+j)+1]
        touchDiffData = touchCaliData - touchRawData
        return touchDiffData

    touchDriveCount,touchSensorCount = channelCheck()
    while(True):
        touchDiffData = abs(getData())
        if np.any(touchDiffData>60000):
            continue
        data[:] = touchDiffData[channelDriveS:channelDrive+channelDriveS,channelSensorS:channelSensor+channelSensorS]




if __name__ == "__main__":
    shared_array = mp.RawArray('i', channelDrive * channelSensor)
    data = np.frombuffer(shared_array, dtype=np.int32).reshape((channelDrive, channelSensor))
    data[:] = np.zeros([channelDrive, channelSensor],dtype=int)

    data_process = mp.Process(target=data_function, args=(shared_array,))
    polyscope_process = mp.Process(target=polyscope_function,args=(shared_array,))
    heatmap_process = mp.Process(target=heatmap_function, args=(shared_array,))
    # camera_process = mp.Process(target=camera_function)

    data_process.start()
    polyscope_process.start()
    heatmap_process.start()
    # camera_process.start()

    # polyscope_process.join()
    # polyscope_process.terminate()
    # data_process.join()
    # heatmap_process.join()
    # camera_process.join()
