# Cloth Simulation Prototype

Este repositório contém um protótipo de simulação de tecido 3D baseado em um modelo massa-mola escrito em Python.

## Estrutura
- `src/mesh3d.py`: cria e gerencia a malha 3D e os graus de liberdade por nó.
- `src/cloth3d.py`: implementa a simulação do tecido com forças elásticas, gravidade e amortecimento.
- `src/draw3d.py`: visualizador em OpenGL com câmera esférica controlada pelo mouse.
- `src/main.py`: ponto de entrada para executar a simulação.

## Como executar
1. Instale as dependências necessárias (por exemplo, `PyOpenGL` e `PyOpenGL_accelerate`).
2. Execute o módulo principal:
   ```bash
   python -m src.main
   ```
   Use as opções `--size`, `--subdivisions`, `--spring`, `--timestep`, `--max-stretch` e `--max-stretch-relaxation` para ajustar a malha e a simulação.
   O parâmetro `--max-stretch` limita o alongamento máximo das molas em relação ao comprimento de repouso (por exemplo, `--max-stretch 1.5`).
   Já `--max-stretch-relaxation` controla o quanto da correção é aplicada por passo (valores menores deixam o tecido mais flexível).
   Os parâmetros `--self-collision-distance` e `--self-collision-iterations` ativam e configuram o sistema de auto colisão, garantindo que o tecido não atravesse a si mesmo.

   O argumento `--scenario` permite alternar entre os cenários disponíveis:
   - `cloth` (padrão): tecido preso nos quatro cantos, suspenso no ar.
   - `sphere`: tecido completamente solto que cai sobre uma esfera sólida imóvel.

   No cenário da esfera também é possível ajustar o raio e a posição inicial do tecido:
   ```bash
   python -m src.main --scenario sphere --sphere-radius 0.75 --drop-height 1.0
   ```

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
