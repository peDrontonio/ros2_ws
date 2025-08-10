# Docker Setup para Projeto PRM - ROS2

Este diretório contém a configuração Docker para democratizar o uso do projeto PRM em diferentes versões do Ubuntu com ROS2.

## 📋 Pré-requisitos

- Docker instalado
- Docker Compose instalado (plugin v2 ou standalone v1)
- Sistema X11 para interface gráfica (Linux)

**Nota**: O script detecta automaticamente se você tem Docker Compose como plugin (`docker compose`) ou standalone (`docker-compose`).

## 🚀 Instalação Rápida do Docker (Ubuntu)

```bash
# Instalar Docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# Instalar Docker Compose
sudo apt-get update
sudo apt-get install docker-compose-plugin

# Reiniciar sessão para aplicar permissões
newgrp docker
```

## 🐳 Versões Disponíveis

### Ubuntu 20.04 + ROS2 Galactic
- **Distribuição**: ROS2 Galactic
- **Gazebo**: Garden
- **Perfil**: `ubuntu20`

### Ubuntu 22.04 + ROS2 Humble
- **Distribuição**: ROS2 Humble
- **Gazebo**: Garden
- **Perfil**: `ubuntu22`

## 🛠️ Como Usar

### 1. Navegar até o diretório Docker
```bash
cd /home/pedrinho/ros2_ws/src/prm/docker
```

### 2. Configurar X11 (para interface gráfica)
```bash
xhost +local:docker
```

### 3. Executar uma versão específica

#### Para Ubuntu 20.04 (ROS2 Galactic):
```bash
docker-compose --profile ubuntu20 up --build -d
docker-compose --profile ubuntu20 exec prm-ubuntu20 bash
```

#### Para Ubuntu 22.04 (ROS2 Humble):
```bash
docker-compose --profile ubuntu22 up --build -d
docker-compose --profile ubuntu22 exec prm-ubuntu22 bash
```

## 🎯 Comandos Úteis no Container

Uma vez dentro do container, você pode usar:

```bash
# Compilar o projeto
colcon build --symlink-install --packages-select prm

# Iniciar simulação completa
ros2 launch prm inicia_simulacao.launch.py

# Testar URDF
ros2 launch prm teste_urdf.launch.py

# Executar controle da tartaruga
ros2 run prm tartaruga

# Executar outros nós
ros2 run prm controle_robo
ros2 run prm robo_mapper
ros2 run prm ground_truth_odometry
```

## 📁 Estrutura do Projeto

```
docker/
├── docker-compose.yml      # Configuração principal
├── Dockerfile.ubuntu20     # Dockerfile para Ubuntu 20.04
├── Dockerfile.ubuntu22     # Dockerfile para Ubuntu 22.04
├── entrypoint.sh          # Script de inicialização
└── README.md              # Este arquivo
```

## 🔧 Personalização

### Modificar Dependências

Para adicionar novas dependências, edite os Dockerfiles correspondentes:
- `Dockerfile.ubuntu20` - Para Ubuntu 20.04
- `Dockerfile.ubuntu22` - Para Ubuntu 22.04

### Configurar Variáveis de Ambiente

Edite o arquivo `entrypoint.sh` para adicionar novas variáveis de ambiente.

### Volumes Persistentes

Os containers usam volumes nomeados para persistir dados:
- `prm_home_ubuntu20` - Dados do usuário Ubuntu 20.04
- `prm_home_ubuntu22` - Dados do usuário Ubuntu 22.04

## 🐛 Resolução de Problemas

### Interface gráfica não funciona
```bash
# Verificar se X11 está habilitado
echo $DISPLAY
xhost +local:docker
```

### Permissões de arquivo
```bash
# Ajustar permissões se necessário
sudo chown -R $USER:$USER /home/pedrinho/ros2_ws/src/prm
```

### Recompilar containers
```bash
# Remover containers e reconstruir
docker-compose down
docker-compose --profile ubuntu20 up --build --force-recreate
# ou
docker-compose --profile ubuntu22 up --build --force-recreate
```

### Limpar sistema Docker
```bash
# Limpar containers parados e imagens não utilizadas
docker system prune -a
```

## 📊 Comparação das Versões

| Característica | Ubuntu 20.04 | Ubuntu 22.04 |
|----------------|---------------|---------------|
| ROS2 Distro    | Galactic      | Humble        |
| Python         | 3.8           | 3.10          |
| Gazebo         | Garden        | Garden        |
| LTS Support    | Até 2025      | Até 2027      |

## 🎯 Casos de Uso Recomendados

### Ubuntu 20.04 (Galactic)
- Compatibilidade com sistemas mais antigos
- Estabilidade comprovada
- Projetos que requerem Python 3.8

### Ubuntu 22.04 (Humble)
- Recursos mais recentes do ROS2
- Melhor performance
- Desenvolvimento ativo e suporte estendido

## 🤝 Contribuição

Para melhorar este setup Docker:

1. Faça um fork do projeto
2. Crie uma branch para sua feature
3. Faça commit das mudanças
4. Envie um pull request

## 📝 Licença

Este projeto está sob a licença Apache-2.0, conforme especificado no arquivo principal do projeto.

## ❓ Suporte

Para dúvidas ou problemas:
1. Verifique a seção de resolução de problemas
2. Consulte a documentação do ROS2
3. Abra uma issue no repositório do projeto
