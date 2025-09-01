function varargout = gui_5(varargin)
% GUI_5 MATLAB code for gui_5.fig
%      GUI_5, by itself, creates a new GUI_5 or raises the existing
%      singleton*.
%
%      H = GUI_5 returns the handle to a new GUI_5 or the handle to
%      the existing singleton*.
%
%      GUI_5('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in GUI_5.M with the given input arguments.
%
%      GUI_5('Property','Value',...) creates a new GUI_5 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before gui_5_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to gui_5_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help gui_5

% Last Modified by GUIDE v2.5 02-Jun-2024 21:03:24

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @gui_5_OpeningFcn, ...
                   'gui_OutputFcn',  @gui_5_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before gui_5 is made visible.
function gui_5_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to gui_5 (see VARARGIN)

% Choose default command line output for gui_5
handles.output = hObject;



f = handles.uipanel2;
X = [1 0.5];
Y = [0.5 0.5];
annotation(f, 'arrow',X,Y);
f = handles.uipanel1;
X = [0 0.5];
Y = [0.5 0.5];
annotation(f, 'arrow',X,Y);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes gui_5 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = gui_5_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in pushbutton_start_obuka.
function pushbutton_start_obuka_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_start_obuka (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global Mdl

load('EEG_obuka.mat');

C3 = EEG_obuka(:,5);
C4 = EEG_obuka(:,6);
klasa = EEG_obuka(:,26);

poceci_pokusaja = [1];
poceci_aktivnih = [];
krajevi_referentnih = [];
krajevi_pokusaja = [];

for i=1:length(klasa) - 1 

    if klasa(i)<klasa(i+1)
    
        poceci_aktivnih = [poceci_aktivnih i + 1];
        krajevi_referentnih = [krajevi_referentnih i];
        
    end
    
    if klasa(i)>klasa(i+1)
        
        krajevi_pokusaja = [krajevi_pokusaja i];
        poceci_pokusaja = [poceci_pokusaja i + 1];
        
    end

end

krajevi_pokusaja = [krajevi_pokusaja length(klasa)];

aktivniC3 = [];
akitvniC4 = [];
prozori_klase = [];
oznake_klase = [];
for i = 1:length(poceci_aktivnih')
    
    aktivniC31 = C3(poceci_aktivnih(i):krajevi_pokusaja(i));
    
    aktivniC3 = [aktivniC3 aktivniC31];

    aktivniC41 = C4(poceci_aktivnih(i):krajevi_pokusaja(i));
    akitvniC4 = [akitvniC4 aktivniC41];
    
    prozori_klase1 = klasa(poceci_aktivnih(i):krajevi_pokusaja(i));
    
    ozake_klasa = unique(prozori_klase1);
    oznake_klase = [oznake_klase ozake_klasa];
end

trazena_snaga_C3_alfa = [];
trazena_snaga_C3_beta = [];
trazena_snaga_C4_alfa = [];
trazena_snaga_C4_beta = [];

for i = 1:size(aktivniC3,2)
    
    [gustina_snage1,f] = snaga(aktivniC3(:,i));
    [trazena_snaga1, frekvencija_talasa1] = nadji_snagu(f,gustina_snage1, 8, 13);
    %gustina_snage_C3_alfa = [gustina_snage_C3_alfa gustina_snage1];
    trazena_snaga_C3_alfa = [trazena_snaga_C3_alfa trazena_snaga1];
    %frekvencija_talasa_C3_alfa = [frekvencija_talasa_C3_alfa frekvencija_talasa1];
    
    [trazena_snaga2, frekvencija_talasa2] = nadji_snagu(f,gustina_snage1, 13, 30);
    trazena_snaga_C3_beta = [trazena_snaga_C3_beta trazena_snaga2];
    
    [gustina_snage2,f1] = snaga(akitvniC4(:,i));
    
    [trazena_snaga3, frekvencija_talasa3] = nadji_snagu(f1,gustina_snage2, 8, 13);
    trazena_snaga_C4_alfa = [trazena_snaga_C4_alfa trazena_snaga3];
    
    [trazena_snaga4, frekvencija_talasa4] = nadji_snagu(f1,gustina_snage2, 13, 30);
    trazena_snaga_C4_beta = [trazena_snaga_C4_beta trazena_snaga4];
    
end
y_C3_alfa = [];
y_C3_beta = [];
y_C4_alfa = [];
y_C4_beta = [];

for i = 1:size(trazena_snaga_C3_alfa,2)
    y1 = rms(trazena_snaga_C3_alfa(:,i));
    y_C3_alfa = [y_C3_alfa y1];
    
    y3 = rms(trazena_snaga_C4_alfa(:,i));
    y_C4_alfa = [y_C4_alfa y3];
end

for i = 1:size(trazena_snaga_C3_beta,2)
    y2 = rms(trazena_snaga_C3_beta(:,i));
    y_C3_beta = [y_C3_beta y2];
    
    y4 = rms(trazena_snaga_C4_beta(:,i));
    y_C4_beta = [y_C4_beta y4];
end

vektor_obelezja = [y_C3_alfa; y_C3_beta; y_C4_alfa; y_C4_beta];

Mdl = fitcdiscr(vektor_obelezja',oznake_klase');

msgbox('Obuka je uspesno izvrsena');


% --- Executes on button press in pushbutton_start_test.
function pushbutton_start_test_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_start_test (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global port 
global uslov
global Mdl
global prediktovane_vrednosti
global stvarne_vrednosti

uslov = 1;
port = serial('COM4');
set(port, 'BaudRate', 921600,'DataBits',8,'StopBit',1, 'FlowControl','Hardware');
port.InputBufferSize = 5000;
fopen(port);
disp('Connected!');

disp('izvrsi otvaranje kanala')
fwrite(port,['>SC;' 255 255 255 '<']);
odgovor = fread(port, 4); 
char(odgovor)
disp('da frekvenciju')
fwrite(port, '>160<');
odgovor = fread(port, 4); 
char(odgovor)
fwrite(port,'>NORMAL<');
odgovor = fread(port, 4); 
char(odgovor)
fwrite(port, '>ON<');
odgovor = fread(port, 4);
char(odgovor)

klase_za_prikaz = load('instrukcije_za_test');
klase_za_prikaz = klase_za_prikaz.instrukcije_za_test;

prikaz = zeros(1,160);
%prikaz1 = zeros(1,160);

broj_prolazaka = 1;
instrukcija = klase_za_prikaz(broj_prolazaka);
podaci_uk = [];
%C3 = [];
%C4 = [];

prediktovane_vrednosti = [];
stvarne_vrednosti = [];
while (uslov)
    
    CH3_vrednosti = [];
    CH4_vrednosti = [];
    klase_vrednostii = [];
    podacii = [];
    broj_bajtova = port.BytesAvailable;
    
    if broj_bajtova>0
        
        podaci = fread(port, broj_bajtova);
        potencijalni_poceci = find(podaci == 62);
        pravi_poceci = [];
        for i = 1:length(potencijalni_poceci)
            trenutni_pocetak = potencijalni_poceci(i);
            if trenutni_pocetak + 82 <= length(podaci)
                if podaci(trenutni_pocetak + 82) == 60
                    pravi_poceci = [pravi_poceci trenutni_pocetak];
                end
            end
        end
        broj_ispravnih = 0;
        for i = 1:length(pravi_poceci)
            trenutni_pocetak = pravi_poceci(i);
            paket = podaci(trenutni_pocetak:trenutni_pocetak + 82);
            
            checksum = 0;
            
            for j = 2:81
                
                checksum = bitxor(checksum, paket(j));
                
            end
            
            if checksum == paket(82)
                
                broj_ispravnih = broj_ispravnih + 1;
                CH3_bajtovi = paket(1 + (5 - 1)*3 + 1: 1 + (5 - 1)*3 + 3);
                CH4_bajtovi = paket(1 + (6 - 1)*3 + 1: 1 + (6 - 1)*3 + 3);
                klase_bajtovi = paket(75:76);
                CH3_vrednost = Bytes2Sample(CH3_bajtovi);
                CH4_vrednost = Bytes2Sample(CH4_bajtovi);
                klase_vrednosti = Bytes2Sample1(klase_bajtovi);
                
                CH3_vrednosti = [CH3_vrednosti, CH3_vrednost];
                CH4_vrednosti = [CH4_vrednosti, CH4_vrednost];
                klase_vrednostii = [klase_vrednostii, klase_vrednosti];
                
            end
            
        end
        
        podacii = [CH3_vrednosti; CH4_vrednosti];
        podaci_uk = [podaci_uk podacii];
        
        %C3 = [C3 CH3_vrednosti];
        %C4 = [C4 CH4_vrednosti];
        
        if broj_prolazaka > 36
            pushbutton_stop_test_Callback(hObject, eventdata, handles);
        else
            
            if size(podaci_uk,2) >= 4.1*160 && rem(broj_prolazaka, 2)==0
              
            
                    [gustina_snage1,f] = snaga(podaci_uk(1,:)');
                    
                    [trazena_snaga1, frekvencija_talasa1] = nadji_snagu(f,gustina_snage1, 8, 13);
                    [trazena_snaga2, frekvencija_talasa2] = nadji_snagu(f,gustina_snage1, 13, 30);

                    [gustina_snage2,f1] = snaga(podaci_uk(2,:)');

                    [trazena_snaga3, frekvencija_talasa3] = nadji_snagu(f1,gustina_snage2, 8, 13);
                    [trazena_snaga4, frekvencija_talasa4] = nadji_snagu(f1,gustina_snage2, 13, 30);
                
                y1 = rms(trazena_snaga1);
                y2 = rms(trazena_snaga2);
                y3 = rms(trazena_snaga3);
                y4 = rms(trazena_snaga4);

                vektor_obelezja = [y1 y2 y3 y4];

                label = predict(Mdl, vektor_obelezja);
              
                
                prediktovane_vrednosti = [prediktovane_vrednosti label];
                stvarne_vrednosti = [stvarne_vrednosti instrukcija];
                
                set(handles.edit2, 'String', num2str(prediktovane_vrednosti));
                set(handles.edit1, 'String', num2str(stvarne_vrednosti));
                
                podaci_uk = [];
                broj_prolazaka = broj_prolazaka + 1

            elseif size(podaci_uk,2) >= 4.2*160 && rem(broj_prolazaka, 2)~=0
          
                podaci_uk = [];
                broj_prolazaka = broj_prolazaka + 1

            end
            
            if instrukcija == 2
                 set(handles.uipanel1, 'backgroundColor', [0 1 0]);
                 set(handles.uipanel2, 'backgroundColor', [1 1 1]);
            elseif instrukcija == 1
                 set(handles.uipanel1, 'backgroundColor', [1 1 1]);
                 set(handles.uipanel2, 'backgroundColor', [0 1 0]);
            elseif instrukcija == 0
                set(handles.uipanel1, 'backgroundColor', [1 1 1]);
                set(handles.uipanel2, 'backgroundColor', [1 1 1]);
            end
            
            instrukcija = klase_za_prikaz(broj_prolazaka);
            
        end
        
        Fs = 160; 
        t = 0:1/Fs:1-1/Fs;
        
        CH1 = get(handles.radiobutton1, 'Value');
        CH2 = get(handles.radiobutton2, 'Value');
        
        if CH1 == 1
           
            prikaz = [prikaz CH3_vrednosti];
            prikaz = prikaz(length(CH3_vrednosti) + 1: end);
   
        else
        
            prikaz = [prikaz CH4_vrednosti];
            prikaz = prikaz(length(CH4_vrednosti) + 1: end);
          
        end
        
        axes(handles.axes1)
        plot(t,prikaz)
        drawnow
     
    end
end


% --- Executes on button press in pushbutton_stop_test.
function pushbutton_stop_test_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton_stop_test (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

global port 
global uslov
global prediktovane_vrednosti
global stvarne_vrednosti

tacnost = 100*sum(prediktovane_vrednosti == stvarne_vrednosti)/length(prediktovane_vrednosti);

set(handles.edit3, 'String', tacnost);

flushinput(port)
fwrite(port, '>OFF<');
pause(0.1);
broj_bajtova = port.BytesAvailable;
odgovor = fread(port, broj_bajtova); % niz bajtova, niz intova
char(odgovor)
% zatvaramo port 
fclose(port);
% brisemo port
clear port;
disp('Disconnected!');

uslov = 0;
fclose(instrfind);



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in radiobutton1.
function radiobutton1_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton1


% --- Executes on button press in radiobutton2.
function radiobutton2_Callback(hObject, eventdata, handles)
% hObject    handle to radiobutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of radiobutton2
