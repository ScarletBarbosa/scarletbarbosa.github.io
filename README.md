# Scarlet Barbosa 
## Graduanda em Análise e Desenvolvimento de Sistemas
 
Scarlet Barbosa, graduanda em Análise e Desenvolvimento de Sistemas (1º semestre), possui experiência com linguagens C++, Matlab e SQL. Também possui conhecimento em HTML5, CSS3. Estuda as seguintes linguagens para ampliar seus conhecimentos:  JavaScript, Java e Python.
 
É formada desde 2018 em Engenharia Cartográfica e de Agrimensura pela Universidade Federal do Paraná, atuando como perita Judicial do Estado do Paraná.
 
# Experiências
## Linguagens de Programação


1. MATLAB;
2. C++;
3. JAVA;
4. Python;
5. JavaScript;
6. HTML5, CSS3;
7. SQL.

## Alguns dos algoritmos desenvolvidos por mim:

### Algoritmo de Criptografia de Palavras (MatLab/Freemat):

O exercício foi proposto como um desafio na disciplina de Segurança da Informação, consiste em criptografar uma frase através de palavras chaves. Abaixo segue o algoritmo que desenvolvi para resolução do desafio.

~~~ matlab
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PROGRAMA CRIPTOGRAFIA | SCARLET BARBOSA %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Software: Freemat Versão 4.2
% Linguagem Utilizada: Freemat/Matlab

% O PROGRAMA DESENVOLVIDO PODE CRIPTOGRAFAR QUALQUER MSG COM QUALQUER FRASE-CHAVE %

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ROTEIRO LÓGICO (para resoluçao do problema):%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% 1º - Extrair letras da chave
% 2º - Tabelar alfabeto criptografado e nao criptografado
% 3º - Criptografar a mensagem

clear all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% 1 - Dados de Entrada %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

alfabeto = 'abcdefghijklmnopqrstuvwxyz'; % alfabeto não criptografado
chave = 'estaciocuritiba'; % frase-chave
msg = 'oexerciciodeveserentregueateaproximasemana'; % mensagem a ser criptografada

ii = 0; % contador iniciando do 0
kk = 0; % contador iniciando do 0
i = length(chave); % extrai o tamanho do vetor 'chave' (15)
j = 1:i; % contador de 1 até 15(i)
k = length(alfabeto); % extrai o tamanho do vetor 'alf2' (26)

a = [1:i]; % cria vetor com 15 posiçoes
b = (1:i); % cria vetor com 15 posiçoes
r = zeros(i,k); 

for c = ii+1:i; % contador que percorrerá o vetor 'a'
    d = kk+1:k; % contador que percorrerá o vetor 'alfabeto'
    chave(c); % cmd que percorre todos as posiçoes do vetor chave
    a = chave; % cmd que armazena os valores do vetor 'chave' no vetor 'a'
    x = (a(c) == alfabeto(d)); % comando que procura a letra no alfabeto (cria vetor de 0 e 1, quando 1 possui a letra de 'a' em determinada posiçao do alfabeto(vetor 'alfabeto')
    indices_x = find(x);   % extraí o indice no alfabeto ( quando 0 entende que não tem, quando 1 entende que tem e armazena o indice)
    b(:,c) = indices_x; % cria vetor com os indices
end    

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% 2 - Elimina valor repetido do vetor %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

t = 1;
n = length(b);

while t<=n
    [y,posicao] = find(b==b(t));
    posicao(1) = [];
    b(posicao) = [];
    n = length(b);
    t = t+1;
end
disp(b)

tam_b = length(b);
alf_chave = alfabeto(b(1:tam_b)) % frase pós extraçao | vetor que acessa a posiçao(indice) das letras no alfabeto nao criptografado

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% 3 - Criar alfabeto criptografado %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

alf_cripto = [b find(alfabeto)]; % concatena os 2 vetores para criar um alfabeto criptografado

tt = 1;
nn = length(alf_cripto);

while tt <= n;
    [yy, posi2] = find(alf_cripto == alf_cripto(tt));
    posi2(1) = [];
    alf_cripto(posi2) =[];
    nn = length(alf_cripto);
    tt = tt+1;
end

disp(alf_cripto);

tam_alf_cripto = length(alf_cripto);
alf_critografado = alfabeto(alf_cripto(1:tam_alf_cripto))

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% 4 - Criptografar a mensagem %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tam_msg = length(msg); % tamanho do vetor msg
vetor_msg = (1:tam_msg); % 
bb = (1:tam_msg);

for cc = ii+1:tam_msg;
    dd = kk+1:k;
    msg(cc);
    vetor_msg = msg;
    xx = (vetor_msg(cc) == alfabeto(dd));
    indices_xx = find(xx);
    bb(:,cc) = indices_xx;
end

disp(bb)

tam_bb = length(bb);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% 5 - Mostrando a mensagem:  %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

msg_original = alfabeto(bb(1:tam_bb)) % mostra a mensagem original
msg_criptografada = alf_critografado(bb(1:tam_bb)) % mostra a mensagem criptografada

~~~

### Algoritmo de Posicionamento Indoor (Matlab/Freemat):

Este algoritmo foi desenvolvido por mim durante meu Trabalho de Conclusão de Curso em Engenharia Cartográfica e de Agrimensura na UFPR (Universidade Federal do Paraná), no segundo semestre de 2017.

![PROPOSTA PARA IMPLEMENTAÇÃO DO MÉTODO FINGERPRINTING PARA POSICIONAMENTO NO UFPR CAMPUSMAP](http://scarletbarbosa.github.io/PROJETO_FINAL_POSICIONAMENTO_INDOOR_SCARLET_BARBOSA.pdf)

O trabalho teve como objetivo propor o posicionamento indoor através do
método "Fingerprinting” utilizando a infraestrutura de rede wi-fi da UFPR como auxílio
para localização e orientação dos usuários do CampusMap. Os objetivos específicos
para a realização deste trabalho seguem os tópicos:
1. Coletar dados RSSI ao longo da área de estudo, com a finalidade de elaborar um
mapa de intensidade wi-fi;
2. Elaborar o algoritmo de posicionamento indoor;
3. Testar e analisar a exatidão do posicionamento obtido com o método proposto
através da discrepância planimétrica.

~~~ matlab
%%%%%%%%%%%%%%%%%%%%
% POSICIONA INDOOR %
%%%%%%%%%%%%%%%%%%%%

clear all
clc

%%%%%%%%%%
% entrada:
%%%%%%%%%%

entrada = [-33 -59 -57];

%%%%%%%%%%%%%%%%%
% consulta mapas:
%%%%%%%%%%%%%%%%%

mapa = dlmread('mapa_recorte.txt');
X = mapa(:,1);
Y = mapa(:,2);
z = mapa(:,3);

[neighbors distances] = kNearestNeighbors(z,entrada(1),1); % FUNÇAO QUE SELECIONA O VALOR IGUAL OU MAIS PRÓXIMO AO DE ENTRADA
e = mapa(neighbors,3);
[i,j] = find(mapa==e);
ii=i(1);
ie=i(end);

mapa2 = dlmread('mapa_recorte_2.txt');
X2 = mapa2(:,1);
Y2 = mapa2(:,2);
z2 = mapa2(:,3);

[neighbors2 distances] = kNearestNeighbors(z2,entrada(2),1);
e2 = mapa2(neighbors2,3);
[i2,j2] = find(mapa2==e2);
ii2=i2(1);
ie2=i2(end);

mapa3 = dlmread('mapa_recorte_3.txt');
X3 = mapa3(:,1);
Y3 = mapa3(:,2);
z3 = mapa3(:,3);

[neighbors3 distances] = kNearestNeighbors(z3,entrada(3),1);
e3 = mapa3(neighbors3,3);
[i3,j3] = find(mapa3==e3);
ii3=i3(1);
ie3=i3(end);

%%%%%%%%%%%%%%%%%%%%%%%
% seleciona coordenadas
%%%%%%%%%%%%%%%%%%%%%%%

xi = mapa(ii:ie,1); % lista de coordenadas 'x'
yi = mapa(ii:ie,2); % lista de coordenadas 'y'
xi2 = mapa2(ii2:ie2,1);
yi2 = mapa2(ii2:ie2,2);
xi3 = mapa3(ii3:ie3,1);
yi3 = mapa3(ii3:ie3,2);

x = mean(xi); % Coordenada 'x' determinada pela media de 'px'
y = mean(yi); % Coordenada 'y' determinada pela media de 'py'
x2 = mean(xi2);
y2 = mean(yi2);
x3 = mean(xi3);
y3 = mean(yi3);

%%%%%%%%%%%%%%%%%%%
% coordenada final:
%%%%%%%%%%%%%%%%%%%

xf = (x+x2+x3)/3;
yf = (y+y2+y3)/3;

%%%%%%%%%%%%%%%%
% exibe posiçao:
%%%%%%%%%%%%%%%%

fprintf('Posiçao(x,y) do SSID:UFPR_SEM_FIO e MAC ADRESS:50-06-04-2B-E9-90 é: x = %f  = e y = %f \n', x, y)
fprintf('Posiçao(x,y) do SSID:LabCartoDgeom e MAC ADRESS:5A-10-8C-2F-D8-C6 é: x = %f  = e y = %f \n', x2, y3)
fprintf('Posiçao(x,y) do SSID:UFPR_SEM_FIO e MAC ADRESS:50-06-04-2B-E7-40 é: x = %f  = e y = %f \n', x3, y3)
fprintf('Posiçao(x,y) calculada pela média é: x = %f  = e y = %f \n', xf, yf)

%%%%%%%%%%%%%%
% comparaçoes:
%%%%%%%%%%%%%%

xr = 677653.992; % coordenada 'x' real no momento do posicionameto
yr = 7183697.091; % coordenada 'x' real no momento do posicionameto


format short
dx = xr - xf % diferença entre a coordenada real e a obtida pelo método
dy = yr - yf % diferença entre a coordenada real e a obtida pelo método

%%%%%%%%%%%%%%%%%%%
% Plotagem do Mapa:
%%%%%%%%%%%%%%%%%%%
 
X = mapa(:,1); % separa a coordenada X
Y = mapa(:,2); % separa a coordenada Y
Z = mapa(:,3); % separa a coordenada Z -> (RSSI)

xi = linspace(min(X),max(X),100); % vetor c/ coordenadas max,min e resoluçao
yi = linspace(min(Y),max(Y),100); % vetor c/ coordenadas max,min e resoluçao

[E N] = meshgrid(xi,yi);
rssi = griddata(X,Y,Z,E,N);
figure
surf(E,N,rssi) % plotagem do mapa

%%%%%%%%%%%%%%%%%%%%%
% Plotagem do Mapa 2:
%%%%%%%%%%%%%%%%%%%%%

X2 = mapa2(:,1); % separa a coordenada X
Y2 = mapa2(:,2); % separa a coordenada Y
Z2 = mapa2(:,3); % separa a coordenada Z -> (RSSI)

xi2 = linspace(min(X2),max(X2),100); % vetor c/ coordenadas max,min e resoluçao
yi2 = linspace(min(Y2),max(Y2),100); % vetor c/ coordenadas max,min e resoluçao

[E N] = meshgrid(xi2,yi2);
rssi = griddata(X2,Y2,Z2,E,N);
figure(2)
surf(E,N,rssi) % plotagem do mapa

%%%%%%%%%%%%%%%%%%%%%
% Plotagem do Mapa 3:
%%%%%%%%%%%%%%%%%%%%%
 
X3 = mapa3(:,1); % separa a coordenada X
Y3 = mapa3(:,2); % separa a coordenada Y
Z3 = mapa3(:,3); % separa a coordenada Z -> (RSSI)

xi3 = linspace(min(X3),max(X3),100); % vetor c/ coordenadas max,min e resoluçao
yi3 = linspace(min(Y3),max(Y3),100); % vetor c/ coordenadas max,min e resoluçao

[E N] = meshgrid(xi3,yi3);
rssi = griddata(X3,Y3,Z3,E,N);
figure(3)
surf(E,N,rssi) % plotagem do mapa

~~~

~~~ matlab

function [neighborIds neighborDistances] = kNearestNeighbors(dataMatrix, queryMatrix, k)
%--------------------------------------------------------------------------
% Program to find the k - nearest neighbors (kNN) within a set of points. 
% Distance metric used: Euclidean distance
% 
% Usage:
% [neighbors distances] = kNearestNeighbors(dataMatrix, queryMatrix, k);
% dataMatrix  (N x D) - N vectors with dimensionality D (within which we search for the nearest neighbors)
% queryMatrix (M x D) - M query vectors with dimensionality D
% k           (1 x 1) - Number of nearest neighbors desired
% 
% Example:
% a = [1 1; 2 2; 3 2; 4 4; 5 6];
% b = [1 1; 2 1; 6 2];
% [neighbors distances] = kNearestNeighbors(a,b,2);
% 
% Output:
% neighbors =
%      1     2
%      1     2
%      4     3
% 
% distances =
%          0    1.4142
%     1.0000    1.0000
%     2.8284    3.0000
%--------------------------------------------------------------------------
neighborIds = zeros(size(queryMatrix,1),k);
neighborDistances = neighborIds;
numDataVectors = size(dataMatrix,1);
numQueryVectors = size(queryMatrix,1);
for i=1:numQueryVectors,
    dist = sum((repmat(queryMatrix(i,:),numDataVectors,1)-dataMatrix).^2,2);
    [sortval sortpos] = sort(dist,'ascend');
    neighborIds(i,:) = sortpos(1:k);
    neighborDistances(i,:) = sqrt(sortval(1:k));
end
~~~
## Alguns dos Mapas desenvolvidos por mim:

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-bioma-brasil-destaque-matinhos.png)

Mapa caracterizando o bioma do Município de Matinhos - PR.

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-bioma-brasil-destaque-parana.png)

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-bioma-brasil-limites-estaduais.png)

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-fitogeografico-ii.png)

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-fitogeografico-iii.png)

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-fitogeografico.png)

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-hidrico-ii.png)

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-hidrico.png)

![Image of Yaktocat](https://scarletbarbosa.github.io/images/mapa-ucs-pr (1).png)
