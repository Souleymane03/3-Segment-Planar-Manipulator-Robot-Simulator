"""
    Programme de simulation d'un robot manipulateur
    pour ce fait le programme comporte 2 partie principales

    #Equation et fonction mathematique permettant de connaître les differents point du robot dans le repère R0

    #Le tracer des graphes et création de la fenêtre principale

"""
#     Importation des module necessaire
import tkinter as tk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from math import radians,cos,sin,sqrt,atan2
import matplotlib.pyplot as plt
import numpy as np
import matplotlib
from time import sleep
from tkinter.messagebox import askyesno



def Matrixs(teta1,teta2,L0,L1,L2):

	T10=np.matrix([[cos(teta1),-sin(teta1),0,L0],[sin(teta1),cos(teta1),0,0],[0,0,1,0],[0,0,0,1]])

	T21=np.matrix([[cos(teta2),-sin(teta2),0,L1],[sin(teta2),cos(teta2),0,0],[0,0,1,0],[0,0,0,1]])

	#matrice passage globale

	T20=np.dot(T10,T21)

	#la matrice position

	A2=np.matrix([[L2],[0],[0],[1]])

    #position de la pince dans R0

	A0=np.dot(T20,A2)

	#X et Y de A dans R0

	AX=A0[0]
	AY=A0[1]


	#les matrices de passage

	T1_0=np.matrix([[cos(teta1),-sin(teta1),0,L0],
                    [sin(teta1),cos(teta1),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])

	T2_1=np.matrix([[cos(teta2),-sin(teta2),0,L1],
                    [sin(teta2),cos(teta2),0,0],
                    [0,0,1,0],
                    [0,0,0,1]])

	X=A0[0]
	Y=A0[1]

	T2_0=np.dot(T1_0,T2_1)



	#la matrice position ,on utilise les coordonnées du A2 dans son repère

	p2=np.matrix([[0],[0],[0],[1]])

	#position de la pince dans R0

	P0=np.dot(T2_0,p2)

	#On recupère X(A2) et Y(A2)

	x0=P0[0]
	y0=P0[1]

	#on met les differents coordonnees dans des variables

	x=[0,L0,x0,X]
	y=[0,0,y0,Y]

	return x,y,X,Y,x0,y0,L0

def parametrageTrace(x,y,X,Y,x0,y0,L0):


    #limite de la graduation

    plt.xlim(-0.5, 5)
    plt.ylim(-0.5,6)


    #inversion des axes du repère

    plt.gca().invert_xaxis()

    #titres des axes

    plt.title("Robot manipulateur planaire")
    plt.ylabel("Abscisse")
    plt.xlabel("Ordonnée")


    #deplacement de l'ordonnée à droite

    plt.gca().yaxis.set_ticks_position('right')

	#tracé le robot

    plt.plot(y,x,'#6f9eaf',markersize=5,lw=10)

	#organe terminal

    plt.scatter([Y-0.0],[X+0.001],marker="$\sqcap$",s=450,c="black",zorder=3)

	#le socle

    plt.scatter([0],[0],marker="s",s=800,c="#6f9eaf",zorder=3)

	#support
    plt.plot([-0.5,10], [-0.4,-0.4], 'o-b', lw=20,c="black")

	#points du robot

    plt.plot(0,L0,'o-b', lw=18 , markersize=25,color="yellow")
    plt.plot(0,L0,'o-b', lw=18 , markersize=10,color="blue")
    plt.plot(y0,x0,'o-b', lw=18 , markersize=25,color="yellow")
    plt.plot(y0,x0,'o-b', lw=18 , markersize=10,color="blue")
    plt.plot(Y,X,'o-b',lw=10,markersize=20,color="white")
    plt.plot(Y,X,'o-b', lw=18 , markersize=10,color="green")

    #robot
    fig.canvas.draw()

#La fonction d'initialisation permettant de mettre le robot en position initiale.
def initialisation():

    #valueurs d'initialisation
    L0=3.5
    L1=3.0
    L2=3.0


    teta1=radians(50)
    teta2=radians(80)

    mat = Matrixs(teta1,teta2,L0,L1,L2)

    parametrageTrace(mat[0],mat[1],mat[2],mat[3],mat[4],mat[5],mat[6])


def deplaceRobot():

	#recuperation des valeurs entrees dans les champs de saisies

    L0=float(entr1_L0.get())
    
    L1=float(entr2_L1.get())
    
    L2=float(entr3_L2.get())
    
    teta1=float(entr3_teta1.get())
    
    teta2=float(entr3_teta2.get())
    
    Xb=float(entr3_b_x.get())
    
    Yb=float(entr3_b_y.get())
    
    NbPas=int(entr3_nb_pas.get())


    teta1=radians(teta1)
    teta2=radians(teta2)

    mat = Matrixs(teta1,teta2,L0,L1,L2)

    AX=mat[2]
    AY=mat[3]
    
	#determination de l'equation de la droite
    a=(Yb-AY)/(Xb-AX)
    b=Yb-a*Xb

	#position initial du robot c'est à dire NbPas=0

    if NbPas==0:

        plt.clf()
        
	#la trajectoire de la pince

        plt.plot([AY,Yb],[AX,Xb],'--',color="#9f5f80" ,lw=3)

        plt.plot(Yb,Xb,'o-b', lw=18 , markersize=10,color="red")

        parametrageTrace(mat[0],mat[1],mat[2],mat[3],mat[4],mat[5],mat[6])

    else:



        PasX=(Xb-AX)/NbPas

	#declaration des listes

        angle1=[]
        angle2=[]
        liste=[]
        table=[]

	#boucle permettant de calculer les differentes coordonnées x_b et y_b  et les stockées dans des listes

        for i in range(1,NbPas+1):

             x_p= AX+i*PasX
             y_p=a*x_p+b


             liste.append(x_p)
             table.append(y_p)


	#boucle permettant de calculer les differents angles et les stockés dans des listes

        for j in range(len(liste)):

             sin_teta1 = [0,0]
             cos_teta1=[0,0]
             teta1_angl=[0,0,]
             W = L2
             X = table[j]
             Y = L0 -liste[j]
             Z1 = 0
             Z2 = -L2
             B1 = 2*(Y*Z1 + X*Z2)
             B2 = 2*(X*Z1 - Y*Z2)
             B3 = W**2 - X**2 - Y**2 - Z1**2 - Z2**2
             e=[1,-1]
             racin_carr =sqrt(B1**2 + B2**2 - B3**2)

             if (B3 != 0) and racin_carr> 0:
                for k in range(1):
                    sin_teta1[k] = (B3*B1 + e[k]*B2*racin_carr)/(B1**2 + B2**2)
                    cos_teta1[k] = (B3*B2 - e[k]*B1*racin_carr)/(B1**2 + B2**2)
                    teta1_angl[k] = atan2(sin_teta1[k],cos_teta1[k])
                    if teta1_angl[k] > 0:
                         angl_trouv_1 = teta1_angl[k]
                    break;
             else:
                   angl_trouv_1 = atan2(-B2,B1)

             X_a = X_b = W
             Y_a = X*cos(angl_trouv_1) + Y*sin(angl_trouv_1) + Z1
             Y_b = X*sin(angl_trouv_1) - Y*sin(angl_trouv_1) + Z2

             angl_trouv_2=atan2(Y_a/X_a, Y_b/X_b)

             angle1.append(angl_trouv_1)
             angle2.append(angl_trouv_2)

	#boucle qui calcule qui determine x et y du robot et tracer en fonction du Pas

    for s in range(NbPas):

		plt.clf()

		T1_0=np.matrix([[cos(angle1[s]),-sin(angle1[s]),0,L0],
					[sin(angle1[s]),cos(angle1[s]),0,0],
					[0,0,1,0],
					[0,0,0,1]])

		T2_1=np.matrix([[cos(angle2[s]),-sin(angle2[s]),0,L1],
					[sin(angle2[s]),cos(angle2[s]),0,0],
					[0,0,1,0],
					[0,0,0,1]])

		X=liste[s]
		Y=table[s]

		#matrice passage globale

		T2_0=np.dot(T1_0,T2_1)

		p2=np.matrix([[0],[0],[0],[1]])

		#position  A2 dans R0

		P0=np.dot(T2_0,p2)
		x0=P0[0]
		y0=P0[1]

		#mettre les x et y du robot dans des variables

		x=[0,L0,x0,X]
		y=[0,0,y0,Y]


		#tracer la trjectoire

		plt.plot([AY,Yb],[AX,Xb],'--',color="#9f5f80" ,lw=3)
		plt.plot(Yb,Xb,'o-b', lw=18 , markersize=10,color="red")
		plt.plot(AY,AX,'o-b', lw=18 , markersize=10,color="red")

		parametrageTrace(x,y,X,Y,x0,y0,L0)

		#pause de 0.1s a chque fois

		sleep(0.1)


#nom de la fenetre

root=tk.Tk()


#Configuration de la fenêtre

root.wm_title("SIMULATEUR DE ROBOT MANIPULATEUR")
root.resizable(False,False)
ecranx=int(root.winfo_screenwidth())
ecrany=int(root.winfo_screenheight())
winx=930
winy=650
posx=(ecranx//2)-(winx//2)
posy=(ecrany//2)-(winy//2)-20



#centré la fenetre

geo="{}x{}+{}+{}".format(winx,winy,posx,posy)
root.geometry(geo)
fig=plt.figure(1)
canvas=FigureCanvasTkAgg(fig,master=root)
root.config(background='#dddddd')


pwidget=canvas.get_tk_widget()
pwidget.place(x=250,y=3)


#fonction permettant de quitter la fenetre

def quitter () :
    if askyesno ("quitter l'application", "voulez-vous quitter ?") :
          root.quit()
          root.destroy()

#fonction permettant d'effacer les champs de saisie

def efface():
  if askyesno ("Suppresion", "voulez-vous supprimer ?") :
    entr1_L0.delete(0,1000)
    entr2_L1.delete(0,1000)
    entr3_teta1.delete(0,1000)
    entr3_teta2.delete(0,1000)
    entr3_b_x.delete(0,1000)
    entr3_b_y.delete(0,1000)
    entr3_nb_pas.delete(0,1000)


#les differents LAbel et Entry

entr1_L0=tk.Entry(root)
Valeur_L0 =tk.Label(root, text ='L0:',bg='#dddddd')

Valeur_L1 =tk.Label(root, text ='L1:' ,bg='#dddddd')
entr2_L1 =tk.Entry(root)
Valeur_L2 =tk.Label(root, text ='L2:',bg='#dddddd')
entr3_L2=tk.Entry(root)
Valeur_teta1 =tk.Label(root, text ='Θ1:',bg='#dddddd')
entr3_teta1=tk.Entry(root)
Valeur_teta2=tk.Label(root, text ='Θ2:',bg='#dddddd')
entr3_teta2=tk.Entry(root)
Valeur_b_x =tk.Label(root, text ='B(x):',bg='#dddddd')
entr3_b_x=tk.Entry(root)
Valeur_b_y =tk.Label(root, text ='B(y):',bg='#dddddd')
entr3_b_y=tk.Entry(root)
Valeur_nb_pas =tk.Label(root, text ='NbrePas:',bg='#dddddd')
Nbre_pas=tk.Label(root, text ='NOMBRE DE PAS A FAIRE',font='bold',bg='#dddddd')
lIENS =tk.Label(root, text ='LONGUEUR DES LIENS',font='bold',bg='#dddddd')
coordonne_b=tk.Label(root, text ='COORDONNEES DU POINT B',font='bold',bg='#dddddd')
Valeur_angle=tk.Label(root,text='VALEURS DES ANGLES',font='bold',bg='#dddddd')
entr3_nb_pas=tk.Entry(root)

# Les differents boutons

BTN_quitter=tk.Button(root,text="Quitter",bg='#ff7b54',borderwidth=3,font='bold',cursor='hand2',command=quitter)

BTN_valider=tk.Button(root,text="Déplacer",bg="#6f9eaf",borderwidth=3,font='bold',cursor='hand2',command=deplaceRobot)

BTN_initial=tk.Button(root,text="Initialiser",bg="#6f9eaf",borderwidth=3,font='bold',cursor='hand2',command=initialisation)

BTN_reset=tk.Button(root,text="Supprimer",bg='#ff7b54',borderwidth=3,font='bold',cursor='hand2',command=efface)

#positionnement des boutons,Label,et Entry sur la fenetre

BTN_valider.place(x=300,y=550,width=150)
BTN_reset.place(x=500,y=550,width=150)
BTN_quitter.place(x=700,y=550,width=150)
BTN_initial.place(x=100,y=550,width=150)

lIENS.place(x=20,y=25)


entr1_L0.place(x=50,y=60,width=100)
Valeur_L0.place(x=30,y=60)
#saisie de L1
entr2_L1.place(x=50,y=100,width=100)
Valeur_L1.place(x=30,y=100)
#saisie de L2
entr3_L2.place(x=50,y=140,width=100)
Valeur_L2.place(x=30,y=140)

#saisie de teta1
Valeur_angle.place(x=20,y=170)
entr3_teta1.place(x=50,y=200,width=100)
Valeur_teta1.place(x=30,y=200)
#saisie de teta2
entr3_teta2.place(x=50,y=250,width=100)
Valeur_teta2.place(x=30,y=250)

coordonne_b.place(x=20,y=300)
#valeur de b_x
entr3_b_x.place(x=60,y=350,width=100)
Valeur_b_x.place(x=30,y=350)
#valeur de b_y
entr3_b_y.place(x=60,y=400,width=100)
Valeur_b_y.place(x=30,y=400)

#nombre de pas
Nbre_pas.place(x=20, y=430)
entr3_nb_pas.place(x=90,y=460,width=100)
Valeur_nb_pas.place(x=30,y=460)


root.mainloop()
