# 🚀 Início Rápido - Docker PRM

## Usar o Script de Conveniência (Recomendado)

```bash
# Entrar no diretório
cd /home/pedrinho/ros2_ws/src/prm/docker

# Ubuntu 22.04 (Recomendado)
./run.sh ubuntu22 run

# Ubuntu 20.04 (Para compatibilidade)
./run.sh ubuntu20 run
```

## Comandos Manuais (Alternativo)

### Ubuntu 22.04 + ROS2 Humble
```bash
cd /home/pedrinho/ros2_ws/src/prm/docker
xhost +local:docker
docker-compose --profile ubuntu22 up --build -d
docker-compose --profile ubuntu22 exec prm-ubuntu22 bash
```

### Ubuntu 20.04 + ROS2 Galactic
```bash
cd /home/pedrinho/ros2_ws/src/prm/docker
xhost +local:docker
docker-compose --profile ubuntu20 up --build -d
docker-compose --profile ubuntu20 exec prm-ubuntu20 bash
```

## Dentro do Container

```bash
# Compilar projeto
colcon build --symlink-install --packages-select prm

# Testar simulação
ros2 launch prm inicia_simulacao.launch.py
```

## Parar Containers

```bash
# Com script
./run.sh ubuntu22 stop

# Manual
docker-compose --profile ubuntu22 down
```

Para mais detalhes, consulte o [README.md](README.md) completo.
