# Cloth Simulation Prototype

Este repositório contém um protótipo de simulação de tecido 3D baseado em um modelo massa-mola. O código Python original foi
mantido e agora também há uma porta completa em C++ utilizando FreeGLUT e OpenMP.

## Estrutura
- `src/mesh3d.py`: cria e gerencia a malha 3D e os graus de liberdade por nó.
- `src/cloth3d.py`: implementa a simulação do tecido com forças elásticas, gravidade e amortecimento.
- `src/draw3d.py`: visualizador em OpenGL com câmera esférica controlada pelo mouse.
- `src/main.py`: ponto de entrada para executar a simulação.
- `cpp/Vec3.h`, `cpp/Mesh3D.*`, `cpp/Cloth3D.*`, `cpp/Draw3D.*`, `cpp/main.cpp`: versão C++ da simulação com renderização via
  FreeGLUT.

## Como executar (Python)
1. Instale as dependências necessárias (por exemplo, `PyOpenGL` e `PyOpenGL_accelerate`).
2. Execute o módulo principal:
   ```bash
   python -m src.main
   ```
   Use as opções `--size`, `--subdivisions`, `--spring`, `--timestep`, `--max-stretch` e `--max-stretch-relaxation` para ajustar a malha e a simulação.
   O parâmetro `--max-stretch` limita o alongamento máximo das molas em relação ao comprimento de repouso (por exemplo, `--max-stretch 1.5`).
   Já `--max-stretch-relaxation` controla o quanto da correção é aplicada por passo (valores menores deixam o tecido mais flexível).
   Os parâmetros `--self-collision-distance` e `--self-collision-iterations` ativam e configuram o sistema de auto colisão, garantindo que o tecido não atravesse a si mesmo.
   Para aproveitar múltiplos núcleos, utilize `--workers` para paralelizar o cálculo das forças de mola (por exemplo, `--workers 4`).

   O argumento `--scenario` permite alternar entre os cenários disponíveis:
   - `cloth` (padrão): tecido preso nos quatro cantos, suspenso no ar.
   - `sphere`: tecido completamente solto que cai sobre uma esfera sólida imóvel.

   No cenário da esfera também é possível ajustar o raio e a posição inicial do tecido:
   ```bash
   python -m src.main --scenario sphere --sphere-radius 0.75 --drop-height 1.0
   ```

## Como compilar e executar (C++)
Você pode compilar manualmente ou utilizar o `make` fornecido no repositório.

### Linux/macOS
1. Instale os pacotes de desenvolvimento do OpenGL/FreeGLUT e o suporte a OpenMP do seu compilador. Em distribuições Debian/Ubuntu
   isso pode ser feito com:
   ```bash
   sudo apt-get install build-essential freeglut3-dev
   ```
2. Compile utilizando o `make` padrão (ou o comando `g++` manual abaixo):
   ```bash
   make
   # ou
   g++ -std=c++17 -O2 -fopenmp cpp/*.cpp -lGL -lGLU -lglut -o cloth_sim
   ```
3. Execute o binário resultante:
   ```bash
   ./cloth_sim --scenario sphere --workers 4
   ```

### Windows (MinGW)
1. Utilize um ambiente MinGW-w64 ou MSYS2 com suporte a OpenMP (`-fopenmp`).
2. As bibliotecas pré-compiladas do FreeGLUT e do OpenGL já estão em `cpp/lib`. Para incluir os cabeçalhos correspondentes, adicione-os
   em `cpp/include` (caso ainda não exista, crie a pasta e copie os headers do FreeGLUT).
3. Compile com o `make` do MinGW, ativando a configuração para Windows:
   ```bash
   mingw32-make PLATFORM=windows
   # ou
   make PLATFORM=windows
   ```
   O alvo gerado será `cloth_sim.exe` e será linkado automaticamente contra os arquivos de `cpp/lib` (`freeglut`, `opengl32`, `glu32`,
   `winmm` e `gdi32`).
4. Execute o binário normalmente:
   ```bash
   cloth_sim.exe --scenario sphere --workers 4
   ```

Os mesmos parâmetros da versão Python estão disponíveis (`--size`, `--subdivisions`, `--max-stretch`, `--self-collision-distance`,
`--workers`, etc.).

## Como enviar as alterações para o GitHub
1. Garanta que suas alterações estejam salvas e verifique o estado do repositório:
   ```bash
   git status
   ```
2. Adicione os arquivos desejados à área de staging:
   ```bash
   git add README.md src/
   ```
3. Crie um commit com uma mensagem descritiva:
   ```bash
   git commit -m "Adiciona README com instruções"
   ```
4. Envie a branch atual para o GitHub:
   ```bash
   git push origin <nome-da-branch>
   ```

Esses passos podem ser executados diretamente a partir deste ambiente ou na sua máquina local vinculada ao repositório remoto.
