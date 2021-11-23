import pybullet as p
import pybullet_data
import time
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

#シミュレーション表示
physicsClient=p.connect(p.GUI)

#pybullet_dataへのファイルパス設定
p.setAdditionalSearchPath(pybullet_data.getDataPath())

#重力を発生させる。地球と同じ重力
p.setGravity(0,0,-10)

#角速度を変更するためのボタン設定
turning=p.addUserDebugParameter("turning",0,.5,0)
fluctuation=p.addUserDebugParameter("fluctuation",0,0.5,0)

#作成したモデルの読み込み
crane=p.loadURDF("AGU_crane/AGU_crane_okano45.urdf")

#配列の初期化
usingforce=1000
mass_position_x=np.zeros(2000)
mass_position_y=np.zeros(2000)
mass_position_z=np.zeros(2000)
boom_fluctuation_velocity=np.zeros(2000)
boom_turning_velocity=np.zeros(2000)
mass_angelspeed=np.zeros(2000)
boom_tip_y=np.zeros(2000)
boom_tip_z=np.zeros(2000)

#シミュレータ動作
for i in range(2000):

    #user_button=p.readUserDebugParameter(button)
    user_turning=p.readUserDebugParameter(turning)
    user_fluctuation=p.readUserDebugParameter(fluctuation)

    #角速度を変更するためのコントローラの読み込み(起伏方向１自由度の場合)
    #p.setJointMotorControl2(crane,1,controlMode=p.VELOCITY_CONTROL,targetVelocity=user_radpersec,force=usingforce)
    #p.setJointMotorControl2(crane,2,controlMode=p.VELOCITY_CONTROL,force=0)

    #角速度を変更するためのコントローラの読み込み(起伏、旋回２自由度の場合)
    p.setJointMotorControl2(crane,1,controlMode=p.VELOCITY_CONTROL,targetVelocity=user_turning,force=usingforce)
    p.setJointMotorControl2(crane,2,controlMode=p.VELOCITY_CONTROL,targetVelocity=user_fluctuation,force=usingforce)
    p.setJointMotorControl2(crane,3,controlMode=p.VELOCITY_CONTROL,force=0)
    p.setJointMotorControl2(crane,4,controlMode=p.VELOCITY_CONTROL,force=0)
    p.setJointMotorControl2(crane,5,controlMode=p.VELOCITY_CONTROL,force=0)

    #吊り荷リンクの位置を表示,linkworldpositionでシミュレーション内での原点からの座標表示
    state=p.getLinkState(crane,6,computeLinkVelocity=1)
    mass_position_x[i]=state[0][0]
    mass_position_y[i]=state[0][1]
    mass_position_z[i]=state[0][2]

    #ブームの先端座標取得
    boom_tip_state=p.getLinkState(crane,3)
    #ブームの先端座標を入力
    boom_tip_y[i]=boom_tip_state[0][1]
    boom_tip_z[i]=boom_tip_state[0][2]

    #吊り荷の固有振動数
    mass_angelspeed[i]=state[7][0]

    #ブームの旋回角速度を表示
    turning_velocity=p.getJointState(crane,1)
    boom_turning_velocity[i]=turning_velocity[1]

    #ブームの起伏角速度を表示
    fluctuation_velocity=p.getJointState(crane,2)
    boom_fluctuation_velocity[i]=fluctuation_velocity[1]


    #シミュレーションを行う
    p.stepSimulation()
    #p.setRealTimeSimulation(1)
    #1/240秒に一回休ませる
    time.sleep(1./240.)

#グラフの描画
x=np.arange(2000)
fig,ax=plt.subplots(3,3,figsize=(14,14))
ax[0,0].plot(x,mass_position_x,label="x")
ax[0,1].plot(x,mass_position_y,label="y")
ax[0,2].plot(x,mass_position_z,label="z")
ax[1,0].plot(x,boom_fluctuation_velocity,label="fluctuation")
ax[1,1].plot(x,boom_turning_velocity,label="turning")
ax[1,2].plot(x,mass_angelspeed,label="mass_AngelSpeed")
ax[2,0].plot(x,boom_tip_y,label="boom_tip_y")
ax[2,1].plot(x,boom_tip_z,label="boom_tip_z")
for i in range(3):
    for t in range(3):
        if i==2 & t==2:
            break
        ax[i,t].legend()
plt.tight_layout()
plt.savefig("AGU_crane_data/AGU_crane_data.jpg")

#データファイル保存(フォーマットはcsv)
df=pd.DataFrame({"mass_position_x":mass_position_x,
                  "mass_position_y":mass_position_y,
                  "mass_position_z":mass_position_z,
                  "boom_turning_velocity":boom_turning_velocity,
                  "boom_fluctuation_velocity":boom_fluctuation_velocity,
                  "mass_Angel_Speed":mass_angelspeed,
                  "boom_tip_y":boom_tip_y,
                  "boom_tip_z":boom_tip_z})
df.to_csv("AGU_crane_data/AGU_crane_data.csv")
