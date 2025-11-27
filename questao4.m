%% Dados do Motor (Modelo 6)
Pn_watt = 111900;   % Potência mecânica (W)
Vn_line = 460;      % Tensão nominal de linha (V)
fn = 60;            % Frequência nominal (Hz)
p = 2;              % Pares de polos

% Parâmetros (Indutâncias e Resistências são constantes)
Rs = 0.0302;          % Resistência estator (Ohms)
Rr_linha = 0.01721;   % Resistência rotor referida (Ohms)
Lls = 0.000283;       % Indutância dispersão estator (H)
Llr_linha = 0.000283; % Indutância dispersão rotor (H)
Lm = 0.01095;         % Indutância mútua (H)

%% Configuração da Simulação V/f
escalas = [1, 0.75, 0.625, 0.5, 0.375, 0.25];
legendas = cell(length(escalas), 1);

% Preparar a figura
fig = figure('Name', 'Torque x Velocidade (V/f Variável)');
hold on;
cores = lines(length(escalas)); % Gerar cores diferentes para cada curva

%% Loop de Cálculo para cada Frequência
for k = 1:length(escalas)
    fator = escalas(k);
    
    % 1. Atualizar Tensão e Frequência Proporcionalmente
    f_op = fn * fator;              % Nova frequência
    V_line_op = Vn_line * fator;    % Nova tensão de linha
    Vph_op = V_line_op / sqrt(3);   % Nova tensão de fase
    
    % 2. Recalcular Velocidade Síncrona e Reatâncias
    % Importante: As reatâncias (X = 2*pi*f*L) mudam com a frequência!
    omega_e_op = 2 * pi * f_op;
    
    Xs_op = omega_e_op * Lls;
    Xr_linha_op = omega_e_op * Llr_linha;
    Xm_op = omega_e_op * Lm;
    
    ns_op = (60 * f_op) / p;        % Nova velocidade síncrona (rpm)
    omega_sinc_op = (2 * pi * ns_op) / 60; % Rad/s
    
    % 3. Vetor de Velocidade para a curva atual
    % Vamos variar de 0 até a velocidade síncrona atual
    nmec_vec = 0:1:ns_op; 
    
    % Inicializar vetor de torque
    Torque_vec = zeros(size(nmec_vec));
    
    % 4. Cálculo do Torque ponto a ponto
    for i = 1:length(nmec_vec)
        nmec = nmec_vec(i);
        
        % Calcular escorregamento para esta velocidade
        s = (ns_op - nmec) / ns_op;
        
        if s == 0
            Torque_vec(i) = 0;
            continue;
        end
        
        % Impedâncias ajustadas para a nova frequência
        Zr = (Rr_linha / s) + 1j*Xr_linha_op;
        Zm = 1j*Xm_op;
        Zpar = (Zm * Zr) / (Zm + Zr);
        Zeq = (Rs + 1j*Xs_op) + Zpar;
        
        % Corrente
        I1 = Vph_op / Zeq;
        
        % Divisor de corrente para I2
        I2 = I1 * (Zm / (Zm + Zr));
        
        % Potência convertida e Torque
        P_conv = 3 * abs(I2)^2 * Rr_linha * (1 - s) / s;
        wmec = omega_sinc_op * (1 - s);
        
        if wmec > 0
            Torque_vec(i) = P_conv / wmec;
        elseif wmec == 0 && s == 1 % Torque de partida
             % Fórmula alternativa direta para partida ou limite
             Torque_vec(i) = (3 * abs(I2)^2 * Rr_linha) / omega_sinc_op;
        end
    end
    
    % 5. Plotar a curva
    plot(nmec_vec, Torque_vec, 'LineWidth', 2, 'Color', cores(k,:));
    
    % Criar texto para legenda
    legendas{k} = sprintf('%.3f Vn, %.3f fn', fator, fator);
end

%% Formatação do Gráfico
grid on;
title('Curvas Torque x Velocidade sob Controle V/f');
xlabel('Velocidade Mecânica (rpm)');
ylabel('Torque (N.m)');
legend(legendas, 'Location', 'northeast');
xlim([0 2000]); % Fixar eixo X para ver o deslocamento das curvas
ylim([0 3500]); % Ajustar conforme o pico máximo do motor

saveas(fig, '4_torque_vs_velocidade_vf_variavel.png'); 

hold off;