function varargout = IntModDig(varargin)
% INTMODDIG MATLAB code for IntModDig.fig
%      INTMODDIG, by itself, creates a new INTMODDIG or raises the existing
%      singleton*.
%
%      H = INTMODDIG returns the handle to a new INTMODDIG or the handle to
%      the existing singleton*.
%
%      INTMODDIG('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INTMODDIG.M with the given input arguments.
%
%      INTMODDIG('Property','Value',...) creates a new INTMODDIG or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before IntModDig_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to IntModDig_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help IntModDig

% Last Modified by GUIDE v2.5 08-Jun-2020 12:09:57

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @IntModDig_OpeningFcn, ...
                   'gui_OutputFcn',  @IntModDig_OutputFcn, ...
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


% --- Executes just before IntModDig is made visible.
function IntModDig_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to IntModDig (see VARARGIN)

% Choose default command line output for IntModDig
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes IntModDig wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = IntModDig_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function dato_Callback(hObject, eventdata, handles)
% hObject    handle to dato (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of dato as text
%        str2double(get(hObject,'String')) returns contents of dato as a double


% --- Executes during object creation, after setting all properties.
function dato_CreateFcn(hObject, eventdata, handles)
% hObject    handle to dato (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in tipoMod.
function tipoMod_Callback(hObject, eventdata, handles)
% hObject    handle to tipoMod (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns tipoMod contents as cell array
%        contents{get(hObject,'Value')} returns selected item from tipoMod


% --- Executes during object creation, after setting all properties.
function tipoMod_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tipoMod (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Fc_Callback(hObject, eventdata, handles)
% hObject    handle to Fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fc as text
%        str2double(get(hObject,'String')) returns contents of Fc as a double


% --- Executes during object creation, after setting all properties.
function Fc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Vc_Callback(hObject, eventdata, handles)
% hObject    handle to Vc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Vc as text
%        str2double(get(hObject,'String')) returns contents of Vc as a double


% --- Executes during object creation, after setting all properties.
function Vc_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Vc (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in GraficarSenal.
function GraficarSenal_Callback(hObject, eventdata, handles)
% hObject    handle to GraficarSenal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject, handles);
bin = reshape(str2num(char(num2cell(dec2bin(get(handles.dato, 'String'))))),1,[]);
%

%
 au = get(handles.dato, 'String');
 A = str2double(get(handles.Vc, 'String'));
 
 
 if isnan(A)  
    f = warndlg('Llenar campo de portadora');
    return;
end

 if strcmp(au,'')==1
    f = warndlg('Llenar campo de ASCII');
    return;
end

switch get(handles.tipoMod,'Value')   
  case 1 
    Vm = [];
    for i=1:length(bin)
        if bin(i) == 1
            Vm(i) = -1;
        else
            Vm(i) = 1;
        end
    end
    
   
    
    
    Wc = 2*pi*str2double(get(handles.Fc, 'String'));
    t1 = linspace(0, 1, 100);
    d = [];
    b = [];
    port = [];
    
    for i=1:length(Vm)
        Vam = (1-Vm(i))*((A/2)*cos(Wc*t1));
        d = [d Vam];
        port = [port A*cos(Wc*t1)];
        if bin(i) == 1
            b = [b ones(1,100)];
        else
            b = [b zeros(1,100)];
        end
    end
    
    figure(1)
    subplot(3,1,1)
    plot(b,'LineWidth', 1.5)
    axis([0	length(bin)*100 -0.5 1.5])
    title('Señal de información')
    grid on;
    subplot(3,1,2)
    plot(port)
    title('Señal portadora')
    grid on;
    subplot(3,1,3)
    plot(d)
    grid on;
    title('Modulación digital en amplitud (ASK)') 
  case 2
    Vc = str2double(get(handles.Vc, 'String'));
    Fc = str2double(get(handles.Fc, 'String'));
    Fm = 2400;
    Fs = 1200;
    deltaF = abs(Fm-Fs)/2; 
    t1 = linspace(0, 1, 100);
    d = [];
    b = [];
    port = [];

    for i=1:length(bin)
        if bin(i) == 1
            b = [b ones(1,100)];
            Vfsk = Vc*cos(2*pi*(Fc+deltaF)*t1);
        else
            b = [b zeros(1,100)];
            Vfsk = Vc*cos(2*pi*(Fc-deltaF)*t1);
        end
        d = [d Vfsk];
        port = [port Vc*cos(2*pi*Fc*t1)];
    end

    figure(1)
    subplot(3,1,1)
    plot(b,'LineWidth', 1.5)
    axis([0 length(bin)*100 -0.5 1.5])
    title('Señal de información')
    grid on;
    subplot(3,1,2)
    plot(port)
    title('Señal portadora')
    grid on;
    subplot(3,1,3)
    plot(d)
    grid on;
    title('Modulación digital en frecuencia (FSK)')
      
  case 3
    Vc = str2double(get(handles.Vc, 'String'));
    Fc = str2double(get(handles.Fc, 'String'));
    t1 = linspace(0, 1, 100);
    d = [];
    b = [];
    port = [];

    for i=1:length(bin)
        if bin(i) == 1
            b = [b ones(1,100)];
            Vbpsk = sin(2*pi*Fc*t1);
        else
            b = [b zeros(1,100)];
            Vbpsk = -sin(2*pi*Fc*t1);
        end
        d = [d Vbpsk];
        port = [port Vc*cos(2*pi*Fc*t1)];
    end
    
    figure(1)
    subplot(3,1,1)
    plot(b,'LineWidth', 1.5)
    axis([0 length(bin)*100 -0.5 1.5])
    title('Señal de información')
    grid on;
    subplot(3,1,2)
    plot(port)
    title('Señal portadora')
    grid on;
    subplot(3,1,3)
    plot(d)
    grid on;
    title('Modulación de desplazamiento de fase binaria (BPSK)')
 otherwise   
    while length(bin) < 8
        bin = [0 bin];
    end
    Vc = str2double(get(handles.Vc, 'String'));
    Fc = str2double(get(handles.Fc, 'String'));
    t1 = linspace(0, 1, 100);

    data=2*bin-1;
    s_p_data=reshape(data,2,length(bin)/2);  

    y=[];
    b=[];
    y_in=[];
    y_qd=[];
    port = [];

    for i=1:length(bin)
        if bin(i) == 1
            b = [b ones(1,100)];
        else
            b = [b zeros(1,100)];
        end
        port = [port Vc*cos(2*pi*Fc*t1)];
    end

    for i=1:length(bin)/2
        y1 = s_p_data(1,i)*cos(2*pi*Fc*t1); 
        y2 = s_p_data(2,i)*sin(2*pi*Fc*t1);
        y_in = [y_in y1]; 
        y_qd = [y_qd y2];
        y = [y y1+y2]; 
    end
    
    figure(1)
    subplot(3,1,1)
    plot(b,'LineWidth', 1.5)
    axis([0 800 -0.5 1.5])
    title('Señal de información')
    grid on;
    subplot(3,1,2)
    plot(port)
    title('Señal portadora')
    grid on;
    subplot(3,1,3)
    plot(y,'linewidth', 1);
    title('Modulación por desplazamiento cuadrafásica (QPSK)');
    grid on;
end 



function sensDesv_Callback(hObject, eventdata, handles)
% hObject    handle to sensDesv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of sensDesv as text
%        str2double(get(hObject,'String')) returns contents of sensDesv as a double


% --- Executes during object creation, after setting all properties.
function sensDesv_CreateFcn(hObject, eventdata, handles)
% hObject    handle to sensDesv (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Fm_Callback(hObject, eventdata, handles)
% hObject    handle to Fm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fm as text
%        str2double(get(hObject,'String')) returns contents of Fm as a double


% --- Executes during object creation, after setting all properties.
function Fm_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fm (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
    warndlg('Pressing ACCEPT clears memory','Warning')
end



function indM_Callback(hObject, eventdata, handles)
% hObject    handle to indM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of indM as text
%        str2double(get(hObject,'String')) returns contents of indM as a double


% --- Executes during object creation, after setting all properties.
function indM_CreateFcn(hObject, eventdata, handles)
% hObject    handle to indM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Fb_Callback(hObject, eventdata, handles)
% hObject    handle to Fb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fb as text
%        str2double(get(hObject,'String')) returns contents of Fb as a double


% --- Executes during object creation, after setting all properties.
function Fb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Fs_Callback(hObject, eventdata, handles)
% hObject    handle to Fs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Fs as text
%        str2double(get(hObject,'String')) returns contents of Fs as a double


% --- Executes during object creation, after setting all properties.
function Fs_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Fs (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function BWMin_Callback(hObject, eventdata, handles)
% hObject    handle to BWMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of BWMin as text
%        str2double(get(hObject,'String')) returns contents of BWMin as a double


% --- Executes during object creation, after setting all properties.
function BWMin_CreateFcn(hObject, eventdata, handles)
% hObject    handle to BWMin (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function deltaF_Callback(hObject, eventdata, handles)
% hObject    handle to deltaF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of deltaF as text
%        str2double(get(hObject,'String')) returns contents of deltaF as a double


% --- Executes during object creation, after setting all properties.
function deltaF_CreateFcn(hObject, eventdata, handles)
% hObject    handle to deltaF (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function tb_Callback(hObject, eventdata, handles)
% hObject    handle to tb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of tb as text
%        str2double(get(hObject,'String')) returns contents of tb as a double


% --- Executes during object creation, after setting all properties.
function tb_CreateFcn(hObject, eventdata, handles)
% hObject    handle to tb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Calcular.
function Calcular_Callback(hObject, eventdata, handles)
% hObject    handle to Calcular (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


guidata(hObject, handles);
Fc = str2double(get(handles.Fc, 'String'));
sensD = str2double(get(handles.sensDesv, 'String'));
Fb = str2double(get(handles.Fb, 'String'));
Fmarca = Fc - sensD;
Fespacio = Fc + sensD;
deltaF = abs(Fmarca - Fespacio)/2;
indiceM = abs(Fmarca - Fespacio)/Fb;


if isnan(sensD)  
    f = warndlg('Llenar campo desviacion');
    return;
end

if isnan(Fc)  
    f = warndlg('Llenar campo de frecuencia portadora');
    return;
end

if isnan(Fb)  
    f = warndlg('Llenar campo de freuencia de bit');
    return;
end

set(handles.deltaF,'String', deltaF);
set(handles.Fm,'String', Fmarca);
set(handles.Fs,'String', Fespacio);
set(handles.indM,'String', indiceM);
set(handles.tb,'String', 1/Fb);

switch get(handles.tipoMod,'Value')
  case 1
    set(handles.BWMin,'String', Fb);    
  case 2
    set(handles.BWMin,'String', 2*(deltaF+Fb));  
  case 3
    set(handles.BWMin,'String', Fb);
    set(handles.condM, 'String', 2);
  otherwise
    set(handles.BWMin,'String', Fb/2);
    set(handles.condM, 'String', 4);
end        

% --- Executes on button press in GraficarEspectro.
function GraficarEspectro_Callback(hObject, eventdata, handles)
% hObject    handle to GraficarEspectro (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject, handles);
Vc = str2double(get(handles.Vc, 'String'));
Fc = str2double(get(handles.Fc, 'String'));
Fm = str2double(get(handles.Fm, 'String'));
Fs = str2double(get(handles.Fs, 'String'));
m = str2double(get(handles.indM, 'String'));

if isnan(Vc)  
    f = warndlg('Llenar campo Portadora');
    return;
end

Bessel1 = FunBessel(m);
Ampl = [];
for i=1:length(Bessel1)
    Ampl(i) = Bessel1(i)*Vc;
end

Amp = fliplr(Ampl); 
Fm1 = Fm:(Fc-Fm)/(length(Bessel1)-1):Fc;
Fs1 = Fc:(Fs-Fc)/(length(Bessel1)-1):Fs;
fre = [Fm1(1:length(Fm1)-1) Fc Fs1(2:length(Fs1))];
y = cat(2,Amp,Ampl(2:length(Amp)));
figure(1)
stem(fre,y)
grid on;
xlabel('Frecuencia (Hz)');
ylabel('Amplitud (V)');
title('Espectro de frecuencias');

function B1 = FunBessel(x)
J1 = zeros(15,1);
for i = 0:14
    J1(i+1,:) = besselj(i,x);
end
B = J1.';
B1 = [];
for j = 1:15
    if abs(B(j)) > 0.01
        B1(j) = abs(B(j));
    end
end

% --- Executes on button press in GraficarConst.
function GraficarConst_Callback(hObject, eventdata, handles)
% hObject    handle to GraficarConst (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject, handles);



 f = warndlg('Ponga la modulacion en BPSK o QPSK');


switch get(handles.tipoMod,'Value')
  case 3
    abscisa=[-1,1];
    ordenada=[0,0];
  case 4
    abscisa=[-1,-1,1,1];
    ordenada=[-1,1,-1,1];
end


figure(1)
a=[-1.5 1.5];
b=a-a;
hold on;
plot(a,b,"k");
plot(b,a,"k");
scatter(abscisa,ordenada,'filled','d');
xlabel('sen(wc*t)');
ylabel('cos(wc*t)');
%points
axis([-1.5 1.5 -1.5 1.5]);
hold off;
grid on;
grid minor;
title('Diagrama de constelación');

%labels
switch get(handles.tipoMod,'Value')
  case 3
    text(-0.98,0,"0")
    text(1.02,0,"1")
  case 4
    text(-1,1,"1,0")
    text(1,1,"1,1")
    text(1,-1,"0,1")
    text(-1,-1,"0,0")
end




function condM_Callback(hObject, eventdata, handles)
% hObject    handle to condM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of condM as text
%        str2double(get(hObject,'String')) returns contents of condM as a double


% --- Executes during object creation, after setting all properties.
function condM_CreateFcn(hObject, eventdata, handles)
% hObject    handle to condM (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over GraficarSenal.
function GraficarSenal_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to GraficarSenal (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
