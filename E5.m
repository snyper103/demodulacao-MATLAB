%Gabriel Alexandre de Souza Braga
clear;
close all;
clc;
%% Carrega sinal AM-DSB na memória %%
[som,Fs] = audioread('fc_5kHz.wav');	% Faz a leitura do audio a ser demodulado

% Fs - freq.  de amostragem [Hz]
% som - matrix de duas colunas (coluna 1 sinal esquerdo, coluna 2 direito)

%% Parametros %%
% Para operações basicas
Ts = 1/Fs;	% Período de amostragem [s]
t = 0:Ts:Ts*(length(som)-1);	% Gera um vetor de tempo
som = som(:,1)';	% De tal maneira o som se torna mono
n = 2^12;
tp = .1;	% Duração do plot

% Para demodulação
Fc = 5000;	% Frequência da portadora
osc = sin((2*pi*Fc*t)+(90*pi/180));	% Oscilador;

% Para analise espectral
fftSinal = fft(som(1:n));	% Transformada rápida de fourier
magSinal = abs(fftSinal);	% Extrai as magnitudes
mag = 2*magSinal(1:n/2)/n;	% Considera somente o semiplano positivo de freqs                               
f = Fs.*(0:(n/2)-1)./n;	% Gera escala de frequências

%% Demodulador %%
demod1 = som.*osc;  % Demodula o sinal
demod1 = lowpass(demod1, 1000, Fs); % Filtra o sinal, retirando o ruido

%% Plot %%
% Cria figura
figure1 = figure('PaperOrientation', 'landscape', 'PaperUnits', 'centimeters',...
    'PaperType', 'A4',...
    'WindowState', 'maximized',...
    'Color', [1 1 1],...
    'Renderer', 'painters');

% Cria primeiro subplot
subplot1 = subplot(2, 2, 1, 'Parent', figure1);
hold(subplot1, 'on');

% Cria plot
plot(t(1:tp*Fs), som(1:tp*Fs), 'DisplayName', 'modulado', 'Parent', subplot1, 'LineWidth', 3,...
     'Color', [0.00 0.45 0.74]);

% Cria rotulo do eixo y e x
ylabel('s_{AM-DSB}(t)', 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('t   (s)', 'FontWeight', 'bold', 'FontName', 'Times New Roman');

% Cria titulo
title('Sinal modulado');

% Define limites do plot, para x e y
xlim(subplot1, [0 0.1]);
ylim(subplot1, [-1.1 1.1]);

% Liga as grades e etc
box(subplot1, 'on');
grid(subplot1, 'on');
hold(subplot1, 'off');

% Define as propriedades restantes dos eixos
set(subplot1, 'AlphaScale', 'log', 'ColorScale', 'log', 'FontName',...
    'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold', 'GridAlpha', 0.5,...
    'LineWidth', 1.5, 'MinorGridAlpha', 0.5);

% Cria segundo subplot
subplot2 = subplot(2,2,2, 'Parent', figure1);
hold(subplot2, 'on');

% Cria plot
plot(f*1e-3,20*log10(mag), 'DisplayName', 'espectro', 'Parent', subplot2, 'LineWidth', 3,...
    'Color', [0.85 0.33 0.10]);

% Cria rotulo do eixo y e x
ylabel('|S_{AM-DSB}(f)|', 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('f   (kHz)', 'FontWeight', 'bold', 'FontName', 'Times New Roman');

% Cria titulo
title('FFT');

% Define limites do plot, para x e y
xlim(subplot2, [0 22]);
ylim(subplot2, [-90 0]);

% Liga as grades e etc
box(subplot2, 'on');
grid(subplot2, 'on');
hold(subplot2, 'off');

% Define as propriedades restantes dos eixos
set(subplot2, 'AlphaScale', 'log', 'ColorScale', 'log', 'FontName',...
    'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold', 'GridAlpha', 0.5,...
    'LineWidth', 1.5, 'MinorGridAlpha', 0.5);

% Cria eixos
axes1 = axes('Parent', figure1,...
    'Position', [0.13 0.11 0.78 0.34]);
hold(axes1, 'on');

% Cria terceiro plot
plot(t(1:tp*Fs),demod1(1:tp*Fs), 'DisplayName', 'demodulado', 'Parent', axes1, 'LineWidth', 3,...
    'Color', [0.49 0.18 0.56]);

% Cria rotulo para o eixo y e x
ylabel('m    (t)', 'FontWeight', 'bold', 'FontName', 'Times New Roman');
xlabel('t   (s)', 'FontWeight', 'bold', 'FontName', 'Times New Roman');

% Cria titulo
title('Sinal demodulado');

% Define limites do plot, para x e y
xlim(axes1, [0 0.1]);
ylim(axes1, [0.33 0.55]);

% Liga as grades e etc
box(axes1, 'on');
grid(axes1, 'on');
hold(axes1, 'off');

% Define as propriedades restantes dos eixos
set(axes1, 'AlphaScale', 'log', 'ColorScale', 'log', 'FontName',...
    'Times New Roman', 'FontSize', 16, 'FontWeight', 'bold', 'GridAlpha', 0.5,...
    'LineWidth', 1.5, 'MinorGridAlpha', 0.5);

%% SOM %%
audio = [som,demod1];   % Assim toca primeiro o sinal modulado, depois o sinal demodulado
soundsc(audio,Fs);  % Toca o sinal modulado e demodulado
