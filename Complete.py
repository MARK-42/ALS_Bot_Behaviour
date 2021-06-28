from tkinter import *
# from tkinter.ttk import *
import random
import time

D_ATT=1
D_REP=500
KATT=4
KREP=3500
V_MAX=6

class BotDetails:
    def __init__(self,idinput,x,y,gx,gy,vx,vy,motionType,staticBehavStatus,dynamicBehavStatus,isplayed):
        self.bid=idinput
        self.bx=x
        self.by=y
        self.bgx=gx
        self.bgy=gy
        self.bvx=vx
        self.bvy=vy
        self.motionType=motionType
        self.staticBehavStatus=staticBehavStatus
        self.dynamicBehavStatus=dynamicBehavStatus
        self.isplayed=False
        
class ObjDetails:
    def __init__(self,idinput,x,y):
        self.oid=idinput
        self.ox=x;
        self.oy=y;

class Simulation:
    
    def __init__(self,canvas):
        self.bots=[]
        self.goals=[]
        self.obstacles=[]
        self.all_coordinates=[]
        self.canvas=canvas
        
    def create_goal(self):
        x=random.randrange(30,1460)
        y=random.randrange(30,760)
        tmplist=[x,y]
        while tmplist in self.all_coordinates:
            x=random.randrange(30,1460)
            y=random.randrange(30,760)
            tmplist=[x,y]
        self.all_coordinates.append(tmplist)
        idinput=self.canvas.create_text(x,y,fill="tomato2",text="X",font=('Helvetica','30','bold'))
        self.goals.append(ObjDetails(idinput, x, y))
    
    def create_bot(self,motiontype,staticBehavStatus,dynamicBehavStatus):
        x=random.randrange(30,1460)
        y=random.randrange(30,760)
        tmplist=[x,y]
        while tmplist in self.all_coordinates:
            x=random.randrange(30,1460)
            y=random.randrange(30,760)
            tmplist=[x,y]
        self.all_coordinates.append(tmplist)
        color=""
        
        if(motiontype=="goal" and staticBehavStatus==0 and dynamicBehavStatus==0):
            color="yellow"
        elif(motiontype=="goal" and staticBehavStatus==1 and dynamicBehavStatus==0):
            color="DarkOrange1"
        elif(motiontype=="goal" and staticBehavStatus==0 and dynamicBehavStatus==1):
            color="tan1"
        elif(motiontype=="goal" and staticBehavStatus==1 and dynamicBehavStatus==1):
            color="salmon1"
        elif(motiontype=="random" and staticBehavStatus==0 and dynamicBehavStatus==0):
            color="cyan"
        elif(motiontype=="random" and staticBehavStatus==1 and dynamicBehavStatus==0):
            color="turquoise1"
        elif(motiontype=="random" and staticBehavStatus==0 and dynamicBehavStatus==1):
            color="aquamarine"
        elif(motiontype=="random" and staticBehavStatus==1 and dynamicBehavStatus==1):
            color="deep sky blue"
        else:
            color="VioletRed1"
        
        idinput=self.canvas.create_oval(x-10,y-10,x+10,y+10,fill=color)
        index = int(random.randrange(0,len(self.goals)))
        gx=self.goals[index].ox
        gy=self.goals[index].oy
        vx=0
        vy=0
        bo=BotDetails(idinput, x, y, gx, gy, vx, vy,motiontype,staticBehavStatus,dynamicBehavStatus,0)
        self.bots.append(bo)
        
    
    
    def create_obstacle_static(self):
        x=random.randrange(30,1460)
        y=random.randrange(30,760)
        tmplist=[x,y]
        while tmplist in self.all_coordinates:
            x=random.randrange(30,1460)
            y=random.randrange(30,760)
            tmplist=[x,y]
        self.all_coordinates.append(tmplist)
        idinput=self.canvas.create_rectangle(x-10,y-10,x+10,y+10,fill="Navajowhite2")
        p=ObjDetails(idinput, x, y)
        self.obstacles.append(p)
        
    def mover(self,isplayed=False):
        # print(len(self.bots))
        # Updating positions
        c=0
        for i in self.bots:
            tb=self.canvas.coords(i.bid)
            i.bx=(tb[0]+tb[2])/2
            i.by=(tb[1]+tb[3])/2
            
            if(isplayed != False):
                i.isplayed=isplayed
                
            if(i.motionType=="follow"):
                if(c==0):
                    px=i.bx
                    py=i.by
                else:
                    i.bgx=px
                    i.bgy=py
                    px=i.bx
                    py=i.by
                c+=1
                
            # print(tb)
        
         #for calculating f_att , frep and velocity for all the bots
        for i in self.bots:
            
            if(i.motionType=="goal" or i.motionType=="follow"):
                #attraction from its respective goal
                d_from_goal3=pow(pow(i.bx-i.bgx,2)+pow(i.by-i.bgy,2),0.5)
                if(d_from_goal3<=D_ATT):
                    F_Att_x3=-KATT*(i.bx-i.bgx)
                    F_Att_y3=-KATT*(i.by-i.bgy)
                else:
                    F_Att_x3=(-KATT*D_ATT*(i.bx-i.bgx))/d_from_goal3
                    F_Att_y3=(-KATT*D_ATT*(i.by-i.bgy))/d_from_goal3
            else:
                # ttt=1
                ch=random.randrange(0,2,1)
                # ch=2
                if(ch==0):
                    F_Att_x3=KATT*random.randrange(-1,1)+0.5
                    F_Att_y3=0
                elif(ch==1):
                    F_Att_x3=0
                    F_Att_y3=KATT*random.randrange(-1,1)+0.5
                else:
                    F_Att_x3=KATT*random.randrange(-1,1)+0.5
                    F_Att_y3=KATT*random.randrange(-1,1)+0.5
                    
            
            #-----------------------------repulsion starts-----------------------
            
            F_Rep_x3=0
            F_Rep_y3=0
            
            if(i.staticBehavStatus==1):
                #repulsion from all static obstacles
                for j in self.obstacles:
                    d_from_rep3=pow(pow(i.bx-j.ox,2)+pow(i.by-j.oy,2),0.5)
                    if(d_from_rep3<=D_REP):
                        dist_mul=(1/d_from_rep3)-(1/D_REP)
                        F_Rep_x3+=(KREP*dist_mul*(i.bx-j.ox))/pow(d_from_rep3,2)
                        F_Rep_y3+=(KREP*dist_mul*(i.by-j.oy))/pow(d_from_rep3,2)
                    else:
                        F_Rep_x3+=0
                        F_Rep_y3+=0
            
           
                
            if(i.dynamicBehavStatus==1):
                # calculating repulsion from all other moving bots
                for k in self.bots:
                    if(k==i):
                        continue
                    d_from_rep3=pow(pow(i.bx-k.bx,2)+pow(i.by-k.by,2),0.5)
                    if(d_from_rep3<=D_REP):
                        dist_mul=(1/d_from_rep3)-(1/D_REP)
                        F_Rep_x3+=(KREP*dist_mul*(i.bx-k.bx))/pow(d_from_rep3,2)
                        F_Rep_y3+=(KREP*dist_mul*(i.by-k.by))/pow(d_from_rep3,2)
                    else:
                        F_Rep_x3+=0
                        F_Rep_y3+=0
   
            F_Res_x3=F_Att_x3+F_Rep_x3
            F_Res_y3=F_Att_y3+F_Rep_y3
            F_Mag3=pow(pow(F_Res_x3,2)+pow(F_Res_y3,2),0.5)
            
            i.bvx=V_MAX*(F_Res_x3/F_Mag3)
            i.bvy=V_MAX*(F_Res_y3/F_Mag3)
        
        
            
        self.movement()
        
        
    def movement(self):
        
        for i in self.bots:
            if(i.isplayed):
                self.canvas.move(i.bid,i.bvx,i.bvy)
        # time.sleep(1)
        # self.mover()
        self.canvas.after(20, self.mover)
        # print("I am here")2

    
class Play:
    def __init__(self,gui):
        self.gui=gui
        
        # add ing canvas widget
        self.canvas = Canvas(self.gui,height=800,width=1500,bg="gray30",borderwidth=5,relief=RAISED)
        self.canvas.grid(row=0,column=0,rowspan=10,padx=5,pady=10)    
        
        #Initialising object for simulation
        self.simulate=Simulation(self.canvas)
        
        #set board 1 goal 1 bot 1 obstacle
        self.simulate.create_goal()
        # self.simulate.create_bot("goal",0,0)
        
        #Adding Label For informations
        self.infoBoard=Label(gui,text="hellooooo",fg="red",font=('Helvetica','22','bold'))
        self.updateLabel()
        self.infoBoard.grid(row=11,column=0,padx=5,pady=10)
        
        #Error
        self.errorLabel=Label(gui,text="",fg="red",font=('Helvetica','16','bold'))
        # self.updateLabel()
        self.errorLabel.grid(row=12,column=0,padx=5,pady=10)
        
        #Goal Adding Widgets
        self.numGoalsText=Text(gui,width=5,height=2,font=('Helvetica','24','bold'))
        self.numGoalsText.grid(row=0,column=1,padx=5,pady=10)
        self.BtnAddGoal=Button(gui,width=15,height=3,text="Add Goal",font=('Helvetica','12','bold'),command=self.AddGoals)
        self.BtnAddGoal.grid(row=0,column=2,padx=5,pady=10)
        
        #Static Obstacles Adding Widgets 
        self.numObstacleText=Text(gui,width=5,height=2,font=('Helvetica','24','bold'))
        self.numObstacleText.grid(row=1,column=1,padx=5,pady=10)
        self.BtnAddObstacle=Button(gui,width=15,height=3,text="Add Obstacle",font=('Helvetica','12','bold'),command=self.AddObstacles)
        self.BtnAddObstacle.grid(row=1,column=2,padx=5,pady=10)
        
        #Bot Adding Widgets
        self.seekingTypeSelected=StringVar()
        self.seekingTypeSelected.set("goal")
        Radiobutton(gui,text="Goal Seeking",variable=self.seekingTypeSelected,
                    value="goal",font=('Helvetica','12','bold')).grid(row=2,column=1,padx=5,pady=10)
        Radiobutton(gui,text="Random Motion",variable=self.seekingTypeSelected ,
                    value="random",font=('Helvetica','12','bold')).grid(row=2,column=2,padx=5,pady=10)
        Radiobutton(gui,text="Follow In Chain",variable=self.seekingTypeSelected ,
                    value="follow",font=('Helvetica','12','bold')).grid(row=3,column=1,columnspan=2,padx=5,pady=10)
        
        self.staticAvoidanceBehavStatus=IntVar()
        self.staticObstacleCHeckBtn=Checkbutton(gui,text="Static Obstacle Avoidance",
                                                variable=self.staticAvoidanceBehavStatus,font=('Helvetica','12','bold'))
        self.staticObstacleCHeckBtn.grid(row=4,column=1,columnspan=2,padx=5,pady=10)
        self.dynamicAvoidanceBehavStatus=IntVar()
        self.dynamicObstacleCHeckBtn=Checkbutton(gui,text="Dynamic Obstacle Avoidance",
                                                  variable=self.dynamicAvoidanceBehavStatus,font=('Helvetica','12','bold'))
        self.dynamicObstacleCHeckBtn.grid(row=5,column=1,columnspan=2,padx=5,pady=10)
        
        self.numBotsText=Text(gui,width=5,height=2,font=('Helvetica','24','bold'))
        self.numBotsText.grid(row=6,column=1,padx=5,pady=10)
        self.BtnAddBots=Button(gui,width=15,height=3,text="Add Bots",font=('Helvetica','12','bold'),command=self.AddBots)
        self.BtnAddBots.grid(row=6,column=2,padx=5,pady=10)
        
        
        #add playButton
        self.start=Button(gui,width=30,height=3,text="Play",font=('Helvetica','12','bold'),command=self.start)
        self.start.grid(row=7,column=1,columnspan=2,padx=5,pady=10)
        
        #add ClearButton
        self.clearBtn=Button(gui,width=30,height=3,text="Clear",font=('Helvetica','12','bold'),command=self.clear)
        self.clearBtn.grid(row=8,column=1,columnspan=2,padx=5,pady=10)
        
    
    def updateLabel(self):
         self.infoBoard.config(text="N-Goals=: "+str(len(self.simulate.goals))+"\t\t"+  
                              "N-Obstacles=: "+str(len(self.simulate.obstacles))+"\t\t"+"N-Bots=: "+
                              str(len(self.simulate.bots))+"\t\t")
         
    def errorUpdate(self,errorMsg):
        self.errorLabel.config(text=errorMsg)
        
    def AddGoals(self):
        txtInput=self.numGoalsText.get("1.0",END).strip()
        if(txtInput == "" or txtInput.isnumeric()==False):
            self.errorUpdate("Please Enter a valid Number")
            return
        numGoals=int(txtInput)
        for i in range(numGoals):
            self.simulate.create_goal()
        self.updateLabel()
        self.numGoalsText.delete(1.0,END)
        
        
    def AddObstacles(self):
        txtInput=self.numObstacleText.get("1.0",END).strip()
        if(txtInput == "" or txtInput.isnumeric()==False):
            self.errorUpdate("Please Enter a valid Number")
            return
        numObstacles=int(txtInput)
        for i in range(numObstacles):
            self.simulate.create_obstacle_static()
        self.updateLabel()
        self.numObstacleText.delete(1.0,END)
    
        
    def AddBots(self):
        txtInput=self.numBotsText.get("1.0",END).strip()
        if(txtInput == "" or txtInput.isnumeric()==False):
            self.errorUpdate("Please Enter a valid Number")
            return
        numBots=int(txtInput)
        for i in range(numBots):
            self.simulate.create_bot(self.seekingTypeSelected.get(),
                                     self.staticAvoidanceBehavStatus.get(),self.dynamicAvoidanceBehavStatus.get())
            
            
        self.updateLabel()
        self.numBotsText.delete(1.0,END)
        # self.playing()
        
    def start(self):
        self.staticObstacleCHeckBtn.deselect()
        self.dynamicObstacleCHeckBtn.deselect()
        self.seekingTypeSelected.set("goal")
        self.simulate.mover(isplayed=True)
        
    def clear(self):
        self.canvas.delete("all")
        self.simulate.__init__(self.canvas)
        self.simulate.create_goal()
        self.staticObstacleCHeckBtn.deselect()
        self.dynamicObstacleCHeckBtn.deselect()
        self.seekingTypeSelected.set("goal")
        # self.simulate.create_bot("goal",0,0)
        self.updateLabel()
        
    
if __name__=="__main__":
    gui=Tk()
    gui.title("BOT Behavior Simulation App")
    gui.geometry("2000x2000")
    player=Play(gui)
    gui.mainloop()