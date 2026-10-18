A = [-0.0156 0.0101 0 -9.81; -0.1367 -1.9635 186 0; 0.000948 -0.02555 -3.6668 0;0 0 1 0];
B = [0; 0.1351; -0.14456; 0];
C = [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
D = [0; 0; 0; 0];
rank_ctrb = rank(ctrb(A, B));
disp(['le rang est ', num2str(rank_ctrb)]);

eigenalues = eig(A);

disp('Valeurs propres de la matrice A :');
disp(eigenalues);

poles_desires = [-1 -2 -3 -4];
K = place(A, B, poles_desires);
disp (['La matrice de gain K est: ', num2str(K)]);

M = A-B*K;
ValP = eig(M);
disp('Valeurs propres de M = A-BK : ')
disp(ValP)