%% 0. Setup
clear; clc; close all;

% Sampling parameters
fs = 48000;           % Sampling frequency (Hz)
dur = 2.0;            % Duration of each tone (seconds)
t = (0:1/fs:dur-1/fs)'; % Time vector

%% Tone parameters
% Tone A parameters (masker - loud tone)
freqA = 1000;       % Frequency of Tone A (1 kHz)
ampA_dB = -6;       % Amplitude of Tone A in dBFS
ampA = 10^(ampA_dB/20);

% Tone B parameters (maskee - soft tone)
freqB = 1500;       % Frequency of Tone B (1.5 kHz)
ampB_dB_list = [-12, -24, -48, -60];  % List of amplitude levels for Tone B in dBFS

% Create fade in/out envelope to avoid clicks (5 ms ramp)
fadeTime = 0.005;            
fadeSamples = round(fadeTime * fs);
fadeEnv = [linspace(0,1,fadeSamples), ones(1, length(t)-2*fadeSamples), linspace(1,0,fadeSamples)]';

%% Generate Tone A (masker)
toneA = ampA * sin(2*pi*freqA*t);
toneA = toneA .* fadeEnv;

%% Spectrogram Parameters
windowSize = 1024;
noverlap = 512;
nfft = 1024;

%% Initialize Storage for Analysis
energyReduction = zeros(length(ampB_dB_list), 1);

%% Loop Over Tone B Amplitude Levels
figure('Name', 'Frequency Masking Analysis', 'NumberTitle', 'off');
hold on;
common_ylim = [-120, 60]; % Set uniform dB range for magnitude plots

for idx = 1:length(ampB_dB_list)
    ampB_dB = ampB_dB_list(idx);
    ampB = 10^(ampB_dB/20);
    
    %% Generate Tone B for current amplitude level
    toneB = ampB * sin(2*pi*freqB*t);
    toneB = toneB .* fadeEnv;
    
    %% Create Combined Signal (Tone A + Tone B)
    combined = toneA + toneB;
    
    %% Compute Spectrograms
    % For Tone B alone
    [S_B, F_B, T_B] = spectrogram(toneB, windowSize, noverlap, nfft, fs);
    magToneB = 20*log10(abs(S_B) + eps);  % Convert to dB
    
    % For Combined signal
    [S_comb, ~, ~] = spectrogram(combined, windowSize, noverlap, nfft, fs);
    magCombined = 20*log10(abs(S_comb) + eps);
    
    % Compute Frequency Spectrum (Averaged Over Time)
    avgSpectrumToneB = mean(magToneB, 2);
    avgSpectrumCombined = mean(magCombined, 2);
    
    % Difference Spectrum (Amount of Masking)
    spectrumDiff = avgSpectrumCombined - avgSpectrumToneB;
    
    % Compute Total Energy Reduction
    energyReduction(idx) = sum((10.^(avgSpectrumToneB/10)) - (10.^(avgSpectrumCombined/10)));
    
    % Plot Overlaid Spectral Reduction
    subplot(2,2,idx);
    plot(F_B, avgSpectrumToneB, 'b', 'LineWidth', 1.5); hold on;
    plot(F_B, avgSpectrumCombined, 'r', 'LineWidth', 1.5);
    title(sprintf('Frequency Masking for Tone B at %ddB', ampB_dB));
    xlabel('Frequency (Hz)'); ylabel('Magnitude (dB)');
    legend('Tone B Alone', 'Masked (Combined)');
    xlim([500 3000]); % Focus on region of interest
    ylim(common_ylim); % Apply the same Y-axis limits
    grid on;
end
sgtitle('Frequency Masking Effect on Different Tone B Levels');

%% Plot Energy Reduction
figure;
bar(ampB_dB_list, energyReduction);
xlabel('Tone B Amplitude (dB)'); ylabel('Total Energy Reduction');
title('Energy Reduction Due to Frequency Masking');
grid on;
