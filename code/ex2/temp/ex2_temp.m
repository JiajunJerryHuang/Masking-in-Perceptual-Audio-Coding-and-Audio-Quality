%% 0. Setup
clear; clc; close all;

% Sampling parameters
fs = 48000;           % Sampling frequency (Hz)
dur = 3.0;            % Duration of each signal (seconds)
t = (0:1/fs:dur-1/fs)'; % Time vector (column vector)

%% Tone parameters
% Tone A (masker - loud tone)
freqA = 1000;      % Frequency of Tone A (1 kHz)
ampA_dB = -3;      % Stronger amplitude for masking
ampA = 10^(ampA_dB/20);

% Tone B (maskee - weak tone)
freqB = 2000;      % Frequency of Tone B (2 kHz)
ampB_dB_list = [-18];  % Test multiple weaker levels for stronger masking
fadeTime = 0.005;  % Fade-in/out time (5 ms)
fadeSamples = round(fadeTime * fs);
fadeEnv = [linspace(0,1,fadeSamples), ones(1, length(t)-2*fadeSamples), linspace(1,0,fadeSamples)]';

% Temporal masking delay settings (in milliseconds)
delays_ms = [10, 50, 100, 300, 500]; % Shorter delays should cause stronger masking

%% Create Output Folder
outputFolder = 'Temporal_Masking_Audio';
if ~exist(outputFolder, 'dir')
    mkdir(outputFolder);
end

%% Loop Over Amplitudes and Delays
for ampB_dB = ampB_dB_list
    ampB = 10^(ampB_dB/20);
    
    for delay_ms = delays_ms
        delay_samples = round((delay_ms / 1000) * fs); % Convert ms to samples
        
        %% Generate Tone A (masker) & Tone B (maskee)
        toneA = ampA * sin(2*pi*freqA*t) .* fadeEnv;
        toneB = ampB * sin(2*pi*freqB*t) .* fadeEnv;
        
        % Apply the delay: Shift Tone B forward in time
        toneB_shifted = [zeros(delay_samples,1); toneB(1:end-delay_samples)];
        
        % Create Combined Signal (Masker + Maskee)
        combined = toneA + toneB_shifted;

        %% Save Audio Files
        originalToneBFile = fullfile(outputFolder, sprintf('ToneB_Only_%ddB.wav', ampB_dB));
        combinedFile = fullfile(outputFolder, sprintf('Masked_ToneB_%dms_%ddB.wav', delay_ms, ampB_dB));

        % Save 16-bit WAV files
        audiowrite(originalToneBFile, toneB, fs, 'BitsPerSample', 16);
        audiowrite(combinedFile, combined, fs, 'BitsPerSample', 16);
        
        fprintf('Saved audio: %s\n', combinedFile);

        %% Compute Spectrograms
        % Spectrogram of Tone B alone
        [S_B, F_B, T_B] = spectrogram(toneB_shifted, 1024, 512, 1024, fs);
        magToneB = 20*log10(abs(S_B) + eps);

        % Spectrogram of Combined Signal
        [S_comb, F_comb, T_comb] = spectrogram(combined, 1024, 512, 1024, fs);
        magCombined = 20*log10(abs(S_comb) + eps);

        % Difference Spectrogram (Highlighting Masked Areas)
        diffSpec = magCombined - magToneB;

        %% Visualization
        figure('Name', sprintf('Temporal Masking (Delay = %dms)', delay_ms), 'NumberTitle', 'off');
        
        % Subplot 1: Time-domain waveform of combined signal
        subplot(3,1,1);
        plot(t, combined, 'b');
        xlabel('Time (s)'); ylabel('Amplitude');
        title(sprintf('Combined Signal (Delay = %dms, AmpB = %ddB)', delay_ms, ampB_dB));
        grid on;

        % Subplot 2: Spectrogram of Combined Signal
        subplot(3,1,2);
        imagesc(T_comb, F_comb, magCombined);
        axis xy; colorbar;
        xlabel('Time (s)'); ylabel('Frequency (Hz)');
        title('Spectrogram of Combined Signal');
        caxis([-80 0]); % Normalize color range

        % Subplot 3: Difference Spectrogram (Masked Areas)
        subplot(3,1,3);
        imagesc(T_B, F_B, diffSpec);
        axis xy; colorbar;
        xlabel('Time (s)'); ylabel('Frequency (Hz)');
        title('Difference Spectrogram (Masked Areas)');
        caxis([-50 50]); % Normalize color range
        
        % Adjust figure layout
        sgtitle(sprintf('Temporal Masking Visualization (Delay = %dms)', delay_ms));
    end
end

fprintf('\nAudio files saved in "%s" folder.\n', outputFolder);
