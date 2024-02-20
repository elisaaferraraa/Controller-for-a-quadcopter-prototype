%PARAMETRI DRONE


%DIMENSIONI (parallelepipedi)
    %masse in kg, lunghezze in metri. 
    %%d1,d2,d3,d4 sono le distanze verticali dal centro di riferimento. d5 è la
    %distanza sull asse x del corpo 4 dall origine. d6 quella dall'asse x del
    %corpo 3.tutte le distanze di sono intese con segno.
    %ai=dimensione lungo l'asse x (width)
    %bi=dimensione lungo l'asse y (length)
    %ci=dimensione lunho l'asse z (height)

    %JETSON
    m1=0.135;
    a1=0.080;
    b1=0.100;
    c1=0.030;
    d1=c1/2; 
    
    %TELAIO
    m2=0.602;
    a2=0.080;
    b2=0.210;
    c2=0.033;
    d2=-c2;
    
    %BATTERIA
    m3=0.228;
    a3=0.035;
    b3=0.105;
    c3=0.033;
    d3=-c2-0.008-c3/2;
    d6=0; %(tra poco lo creo)
    
    %REALSENSE
    m4=0.055;
    a4=0.08;
    b4=0.0125;
    c4=0.0245;
    d4=-c2;
    d5=b2/2+0.025+b4/2;



%MOTORI
    %DIMENSIONI MOTORI (cilindri)
    m=0.055;
    h=0.017;
    r=0.016;
    %dx= 0.013  
    %dy=0.0115
    dx=0.035/(2*sqrt(2));
    dy=0.035/(2*sqrt(2)); %nella simulazione si considera configurazione a X quadrata

    %COSTANTI
    Ct=0.015782 ; %adimensionale N/(rad/s)^2
    Cq=0.002508; %adimensionale
    RPMmax= 22449; 
    wmax=RPMmax*(2*pi)/60;

    %ELICHE 
    re=0.0889; %raggio eliche

    
%CENTRO DI MASSA E d6
    [COM,d6]=CentreOfMass(m1,m2,m3,m4,m,d1,d2,d3,d4,d5,dx,dy,h);

%MATRICE DI INERZIA 
    I=Inertial_Matrix_DART(m1, m2, m3, m4, m, d1, d2, d3, d4, d5, dx, dy, r, h, a1, b1, c1, a2, b2, c2, a3, b3, c3, a4, b4, c4);

%CENTRO DI SPINTA (m)
    %CDS. il centro di spinta si trova ad una quota data dall'altezza dei motori
    %d = distanza tra centro di massa e centro di spinta utile per il calcolo del momento generato dalla forza di gravità
        CDS=h;
        d=CDS-COM(3);

%MASSA TOTALE (kg)
    M=m1+m2+m3+m4+4*m;
       
%COSTANTI AMBIENTE
    rho=1.184; %(kg/m^3)
    g=9.81; %(m/s^2)
     ct=rho*(pi*re^2)*(re^2)*Ct; %N/(rad/s)^2
     cq=-rho*(pi*re^2)*(re^2)*Cq; %Nm/(rad/s)^2


