%% Dados do Motor (Modelo 6)
Pn_watt = 111900;   % Potência mecânica nominal (W)
Vn_line = 460;      % Tensão nominal de linha (V)
fn = 60;            % Frequência nominal (Hz)
p = 2;              % Pares de polos

% Parâmetros do Circuito Equivalente 
Rs = 0.0302;        % Resistência do estator (Ohms)
Lls = 0.000283;     % Indutância de dispersão do estator (H)
Rr_linha = 0.01721; % Resistência do rotor referida (Ohms)
Llr_linha = 0.000283; % Indutância de dispersão do rotor (H)
Lm = 0.01095;       % Indutância mútua (H)

%% 1. Cálculos Preliminares
omega_e = 2 * pi * fn;          % Frequência angular elétrica (rad/s)
Xs = omega_e * Lls;             % Reatância do estator
Xr_linha = omega_e * Llr_linha; % Reatância do rotor
Xm = omega_e * Lm;              % Reatância de magnetização
Vph = Vn_line / sqrt(3);        % Tensão de fase (V)
n_sinc = (60 * fn) / p;         % Velocidade síncrona (rpm)
omega_sinc = (2 * pi * n_sinc) / 60; % Vel. síncrona (rad/s)

%% 2. Definição do Ponto Nominal
% Vamos usar o escorregamento nominal calculado no Cap 2 do relatório: 1.02%
s_nom = 0.0102;

% Função auxiliar para calcular circuito
calc_motor = @(s, Vph, Rs, Xs, Rr_linha, Xr_linha, Xm, omega_sinc) ...
    deal( ...
        Vph ./ ((Rs + 1j*Xs) + (1j*Xm .* (Rr_linha./s + 1j*Xr_linha)) ./ (1j*Xm + (Rr_linha./s + 1j*Xr_linha))), ... % I1 (Fasor)
        (3 * abs(Vph ./ ((Rs + 1j*Xs) + (1j*Xm .* (Rr_linha./s + 1j*Xr_linha)) ./ (1j*Xm + (Rr_linha./s + 1j*Xr_linha)))).^2 .* (abs(1j*Xm ./ (1j*Xm + (Rr_linha./s + 1j*Xr_linha)))).^2 .* (Rr_linha./s)) ./ omega_sinc ... % Torque
    );

% Calcular valores nominais de referência
[I1_nom_phasor, T_nom] = calc_motor(s_nom, Vph, Rs, Xs, Rr_linha, Xr_linha, Xm, omega_sinc);
I1_nom = abs(I1_nom_phasor);

%% 3. Questão 3 - Parte A e B (Curvas Completas)
% Vetor de escorregamento completo
s_full = 0.001:0.001:1;

% Inicializar vetores
I1_full_abs = zeros(size(s_full));
Torque_full = zeros(size(s_full));
nmec_full = n_sinc * (1 - s_full);      % Velocidade em RPM
wmec_full = omega_sinc * (1 - s_full);  % Velocidade em rad/s

% Loop de cálculo para curva completa
for k = 1:length(s_full)
    % Impedância do Rotor
    Zr = (Rr_linha / s_full(k)) + 1j*Xr_linha;
    % Impedância de Magnetização
    Zm = 1j*Xm;
    % Impedância Paralela (Rotor // Magnetização)
    Zpar = (Zm * Zr) / (Zm + Zr);
    % Impedância Equivalente Total
    Zeq = (Rs + 1j*Xs) + Zpar;
    
    % Corrente do Estator (Fasor)
    I1 = Vph / Zeq;
    I1_full_abs(k) = abs(I1);
    
    % Divisor de corrente para achar I2 (Corrente do rotor)
    I2 = I1 * (Zm / (Zm + Zr));
    
    % Potência convertida e Torque
    P_conv = 3 * abs(I2)^2 * Rr_linha * (1 - s_full(k)) / s_full(k);
    if s_full(k) == 0 % Evitar divisão por zero se s=0
       Torque_full(k) = 0;
    else
       Torque_full(k) = P_conv / wmec_full(k);
    end
end

% Plotagem Curvas (a) e (b)
fig0 = figure('Name', 'Torque x Velocidade Mecânica');
plot(nmec_full, Torque_full, 'LineWidth', 1.5);
grid on;
xlabel('n_{mec} (rpm)'); ylabel('Torque (N.m)');
title('Torque x Velocidade Mecânica');
saveas(fig0, 'a_torque_vs_velocidade.png');

fig1 = figure('Name', 'Corrente Estator x Velocidade Mecânica');
plot(nmec_full, I1_full_abs, 'r', 'LineWidth', 1.5);
grid on;
xlabel('n_{mec} (rpm)'); ylabel('|I_1| (A)');
title('Corrente Estator x Velocidade Mecânica');
saveas(fig1, 'b_correnteestator_vs_velocidade.png');

%% 4. Questão 3 - Parte C a F (Região de Operação)
% Encontrar Torque Máximo e Escorregamento Correspondente
[T_max, idx_max] = max(Torque_full);
s_Tmax = s_full(idx_max);

fprintf('Torque Máximo (Tmax): %.2f N.m em s = %.4f\n', T_max, s_Tmax);

% Novo vetor de escorregamento
s_op = 0.001:0.001:s_Tmax;

% Inicializar vetores de operação
eta_op = zeros(size(s_op)); % Eficiência
fp_op = zeros(size(s_op));  % Fator de potência
Pmec_op = zeros(size(s_op)); % Potência Mecânica de Saída
I1_op_abs = zeros(size(s_op));

for k = 1:length(s_op)
    s = s_op(k);
    nmec = n_sinc * (1 - s);
    wmec = omega_sinc * (1 - s);
    
    % Recálculo das impedâncias
    Zr = (Rr_linha / s) + 1j*Xr_linha;
    Zm = 1j*Xm;
    Zpar = (Zm * Zr) / (Zm + Zr);
    Zeq = (Rs + 1j*Xs) + Zpar;
    
    % Correntes
    I1 = Vph / Zeq;
    I1_op_abs(k) = abs(I1);
    I2 = I1 * (Zm / (Zm + Zr));
    
    % Potências
    Pin = 3 * real(Vph * conj(I1)); % Potência ativa de entrada
    P_conv = 3 * abs(I2)^2 * Rr_linha * (1 - s) / s;
    
    % Considerando P_mec = P_conv (desprezando perdas rotacionais)
    Pmec_op(k) = P_conv; 
    
    % Eficiência e Fator de Potência
    eta_op(k) = (Pmec_op(k) / Pin) * 100; % Em %
    fp_op(k) = cos(angle(Vph) - angle(I1));
end

% Plotagem Curvas (c) a (f)
fig2 = figure('Name', 'Eficiência x P_{mec}');
plot(Pmec_op, eta_op, 'LineWidth', 1.5); grid on;
xlabel('P_{mec} (W)'); ylabel('Eficiência \eta (%)'); title('Eficiência x P_{mec}');
saveas(fig2, 'c_eficiencia_vs_pmec.png');

fig3 = figure('Name', 'Fator de Potência x P_{mec}');
plot(Pmec_op, fp_op, 'LineWidth', 1.5); grid on;
xlabel('P_{mec} (W)'); ylabel('Fator de Potência'); title('Fator de Potência x P_{mec}');
saveas(fig3, 'd_fp_vs_pmec.png');

fig4 = figure('Name', 'Escorregamento x P_{mec}');
plot(Pmec_op, s_op, 'LineWidth', 1.5); grid on;
xlabel('P_{mec} (W)'); ylabel('Escorregamento s'); title('Escorregamento x P_{mec}');
saveas(fig4, 'e_s_vs_pmec.png');

fig5 = figure('Name', 'Corrente Estator x P_{mec}');
plot(Pmec_op, I1_op_abs, 'LineWidth', 1.5); grid on;
xlabel('P_{mec} (W)'); ylabel('|I_1| (A)'); title('Corrente Estator x P_{mec}');
saveas(fig5, 'f_corrente_vs_pmec.png');

%% 5. Questão 3 - Dados Tabulados (G a O)
% Partida (s = 1)
[I_start_phasor, T_start] = calc_motor(1, Vph, Rs, Xs, Rr_linha, Xr_linha, Xm, omega_sinc);
I_start_abs = abs(I_start_phasor);

% Torque Máximo (já calculado anteriormente como T_max e s_Tmax)
% Corrente no Torque Máximo
[I_Tmax_phasor, ~] = calc_motor(s_Tmax, Vph, Rs, Xs, Rr_linha, Xr_linha, Xm, omega_sinc);
I_Tmax_abs = abs(I_Tmax_phasor);

% Exibir Tabela no Command Window
fprintf('\n--- Tabela de Resultados (g a o) ---\n');
fprintf('(g) Corrente de partida (|I1,p|): %.2f A\n', I_start_abs);
fprintf('(h) Razão Ip/Inom: %.2f\n', I_start_abs / I1_nom);
fprintf('(i) Torque de partida (Tp): %.2f N.m\n', T_start);
fprintf('(j) Razão Tp/Tnom: %.2f\n', T_start / T_nom);
fprintf('(k) Corrente no Torque Máximo: %.2f A\n', I_Tmax_abs);
fprintf('(l) Razão I_Tmax/Inom: %.2f\n', I_Tmax_abs / I1_nom);
fprintf('(m) Torque Máximo (Tmax): %.2f N.m\n', T_max);
fprintf('(n) Escorregamento de Tmax: %.4f\n', s_Tmax);
fprintf('(o) Razão Tmax/Tnom: %.2f\n', T_max / T_nom);

%% 6. Questão 3 - A Vazio (P a Q)
s_vazio = 0.001;

% Recalcular circuito para s_vazio
Zr_vazio = (Rr_linha / s_vazio) + 1j*Xr_linha;
Zm_vazio = 1j*Xm;
Zpar_vazio = (Zm_vazio * Zr_vazio) / (Zm_vazio + Zr_vazio);
Zeq_vazio = (Rs + 1j*Xs) + Zpar_vazio;

I1_vazio_phasor = Vph / Zeq_vazio;
I1_vazio_abs = abs(I1_vazio_phasor);
I1_vazio_angle_deg = angle(I1_vazio_phasor) * (180/pi);
fp_vazio = cos(angle(Vph) - angle(I1_vazio_phasor));

fprintf('\n--- Operação a Vazio (p a q) ---\n');
fprintf('(p) Fasor Corrente a Vazio: %.2f /_ %.2f° A\n', I1_vazio_abs, I1_vazio_angle_deg);
fprintf('(q) Fator de Potência a Vazio: %.4f\n', fp_vazio);