# imports every file form tkinter and tkinter.ttk
from tkinter import *
from tkinter.ttk import *
import random


D_ATT=1
D_REP=500
KATT=4
KREP=3500
V_MAX=6

class ObjDetails:
    def __init__(self,idinput,x,y):
        self.oid=idinput
        self.ox=x;
        self.oy=y;

class GFG:
    def __init__(self,master):
        self.master = master
        self.x = 1
        self.y = 0
        self.xg=1
        self.yg=0
        
        self.canvas = Canvas(master,height=2000,width=2000)
        self.curr_x=100
        self.curr_y=800
        self.g_x=100
        self.g_y=800
        self.curr_r=20
        self.q = self.canvas.create_oval(100-10, 800-10, 100+10, 800+10, fill = "blue")
        
        self.final_x=1400
        self.final_y=400
        self.goal_r=10
        self.goal = self.canvas.create_oval(1400-10,400-10,1400+10,400+10,fill="red")
        
        
        
        
        self.obstacle_x=(self.curr_x+self.final_x)/2
        self.obstacle_y=(self.curr_y+self.final_y)/2
        self.obstacle_r=10
        self.obs = self.canvas.create_oval(self.obstacle_x-self.obstacle_r,self.obstacle_y-self.obstacle_r,self.obstacle_x+self.obstacle_r,self.obstacle_y+self.obstacle_r,fill="black")
        
        self.obstacles=[]
        for i in range(200):
            x=random.randrange(0,1800)
            y=random.randrange(0,1000)
            idinput=self.canvas.create_rectangle(x-10,y-10,x+10,y+10,fill="black")
            p=ObjDetails(idinput, x, y)
            self.obstacles.append(p)
        
        
        self.mover()
        self.canvas.pack()
        
        
    
    def mover(self):
        t=self.canvas.coords(self.q)
        self.curr_x=(t[0]+t[2])/2
        self.curr_y=(t[1]+t[3])/2
        tg=self.canvas.coords(self.goal)
        self.currg_x=(tg[0]+tg[2])/2
        self.currg_y=(tg[1]+tg[3])/2
        print(t)
        # F_Att_x=random.randrange(-10,10)
        # F_Att_y=random.randrange(-10,10)
        d_from_goal=pow(pow(self.curr_x-self.final_x,2)+pow(self.curr_y-self.final_y,2),0.5)
        if(d_from_goal<=D_ATT):
            F_Att_x=-KATT*(self.curr_x-self.final_x)
            F_Att_y=-KATT*(self.curr_y-self.final_y)
        else:
            F_Att_x=(-KATT*D_ATT*(self.curr_x-self.final_x))/d_from_goal
            F_Att_y=(-KATT*D_ATT*(self.curr_y-self.final_y))/d_from_goal
   
        d_from_rep=pow(pow(self.curr_x-self.obstacle_x,2)+pow(self.curr_y-self.obstacle_y,2),0.5)
        if(d_from_rep<=D_REP):
            dist_mul=(1/d_from_rep)-(1/D_REP)
            F_Rep_x=(KREP*dist_mul*(self.curr_x-self.obstacle_x))/pow(d_from_rep,2)
            F_Rep_y=(KREP*dist_mul*(self.curr_y-self.obstacle_y))/pow(d_from_rep,2)
        else:
            F_Rep_x=0
            F_Rep_y=0
            
        for i in self.obstacles:
            d_from_rep=pow(pow(self.curr_x-i.ox,2)+pow(self.curr_y-i.oy,2),0.5)
            if(d_from_rep<=D_REP):
                dist_mul=(1/d_from_rep)-(1/D_REP)
                F_Rep_x+=(KREP*dist_mul*(self.curr_x-i.ox))/pow(d_from_rep,2)
                F_Rep_y+=(KREP*dist_mul*(self.curr_y-i.oy))/pow(d_from_rep,2)
            else:
                F_Rep_x+=0
                F_Rep_y+=0
        
        d_from_rep=pow(pow(self.curr_x-self.currg_x,2)+pow(self.curr_y-self.currg_y,2),0.5)
        if(d_from_rep<=D_REP):
            dist_mul=(1/d_from_rep)-(1/D_REP)
            F_Rep_x+=(KREP*dist_mul*(self.curr_x-self.currg_x))/pow(d_from_rep,2)
            F_Rep_y+=(KREP*dist_mul*(self.curr_y-self.currg_y))/pow(d_from_rep,2)
        else:
            F_Rep_x+=0
            F_Rep_y+=0
   
        F_Res_x=F_Att_x+F_Rep_x
        F_Res_y=F_Att_y+F_Rep_y

        F_Mag=pow(pow(F_Res_x,2)+pow(F_Res_y,2),0.5)

        self.x=V_MAX*(F_Res_x/F_Mag)
        self.y=V_MAX*(F_Res_y/F_Mag)
        
        #goal's pid
        
        d_from_goal2=pow(pow(self.currg_x-self.g_x,2)+pow(self.currg_y-self.g_y,2),0.5)
        if(d_from_goal2<=D_ATT):
            F_Att_x2=-KATT*(self.currg_x-self.g_x)
            F_Att_y2=-KATT*(self.currg_y-self.g_y)
        else:
            F_Att_x2=(-KATT*D_ATT*(self.currg_x-self.g_x))/d_from_goal2
            F_Att_y2=(-KATT*D_ATT*(self.currg_y-self.g_y))/d_from_goal2
            
        
        d_from_rep2=pow(pow(self.currg_x-self.obstacle_x,2)+pow(self.currg_y-self.obstacle_y,2),0.5)
        if(d_from_rep2<=D_REP):
            dist_mul=(1/d_from_rep2)-(1/D_REP)
            F_Rep_x2=(KREP*dist_mul*(self.currg_x-self.obstacle_x))/pow(d_from_rep2,2)
            F_Rep_y2=(KREP*dist_mul*(self.currg_y-self.obstacle_y))/pow(d_from_rep2,2)
        else:
            F_Rep_x2=0
            F_Rep_y2=0
            
        
        for i in self.obstacles:
            d_from_rep2=pow(pow(self.currg_x-i.ox,2)+pow(self.currg_y-i.oy,2),0.5)
            if(d_from_rep2<=D_REP):
                dist_mul=(1/d_from_rep2)-(1/D_REP)
                F_Rep_x2+=(KREP*dist_mul*(self.currg_x-i.ox))/pow(d_from_rep2,2)
                F_Rep_y2+=(KREP*dist_mul*(self.currg_y-i.oy))/pow(d_from_rep2,2)
            else:
                F_Rep_x2+=0
                F_Rep_y2+=0
        
        d_from_rep2=pow(pow(self.currg_x-self.curr_x,2)+pow(self.currg_y-self.curr_y,2),0.5)
        if(d_from_rep2<=D_REP):
            dist_mul=(1/d_from_rep2)-(1/D_REP)
            F_Rep_x2+=(KREP*dist_mul*(self.currg_x-self.curr_x))/pow(d_from_rep2,2)
            F_Rep_y2+=(KREP*dist_mul*(self.currg_y-self.curr_y))/pow(d_from_rep2,2)
        else:
            F_Rep_x2+=0
            F_Rep_y2+=0
            
        F_Res_x2=F_Att_x2+F_Rep_x2
        F_Res_y2=F_Att_y2+F_Rep_y2

        F_Mag2=pow(pow(F_Res_x2,2)+pow(F_Res_y2,2),0.5)

        self.xg=V_MAX*(F_Res_x2/F_Mag2)
        self.yg=V_MAX*(F_Res_y2/F_Mag2)
            
        self.movement()
        
        
        
    
    def movement(self):
        print("-------")
        print(self.x)
        print(self.y)
        print("-------")
        self.canvas.move(self.q, self.x, self.y)
        self.canvas.move(self.goal, self.xg, self.yg)
        self.canvas.after(30, self.mover)
        


if __name__=="__main__":
    master=Tk()
    master.geometry("2000x2000")
    gfg=GFG(master)
    
    master.mainloop()
    


