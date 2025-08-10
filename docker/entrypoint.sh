#!/bin/bash

# Script de entrada para containers Docker do projeto PRM

# Função para detectar a distribuição ROS2
detect_ros_distro() {
    if [ -f "/opt/ros/galactic/setup.bash" ]; then
        echo "galactic"
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        echo "humble"
    else
        echo "unknown"
    fi
}

# Detectar distribuição ROS2
ROS_DISTRO=$(detect_ros_distro)
echo "🤖 Detectada distribuição ROS2: $ROS_DISTRO"

# Source do ROS2
if [ "$ROS_DISTRO" != "unknown" ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
    echo "✅ ROS2 $ROS_DISTRO carregado com sucesso"
else
    echo "❌ Erro: Distribuição ROS2 não reconhecida"
fi

# Configurar variáveis de ambiente do Gazebo
export GZ_VERSION=garden
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/ros/ros2_ws/src/prm/models

# Verificar se o workspace existe e criar se necessário
if [ ! -d "/home/ros/ros2_ws" ]; then
    echo "📁 Criando workspace ROS2..."
    mkdir -p /home/ros/ros2_ws/src
fi

# Verificar se o pacote PRM está montado
if [ -d "/ros2_ws/src/prm" ]; then
    echo "📦 Criando link simbólico para o pacote PRM..."
    ln -sf /ros2_ws/src/prm /home/ros/ros2_ws/src/prm
    echo "✅ Pacote PRM vinculado com sucesso"
else
    echo "⚠️  Aviso: Pacote PRM não encontrado em /ros2_ws/src/prm"
fi

# Entrar no diretório do workspace
cd /home/ros/ros2_ws

# Compilar o workspace se o pacote PRM estiver presente
if [ -d "/home/ros/ros2_ws/src/prm" ]; then
    echo "🔨 Compilando workspace..."
    colcon build --symlink-install --packages-select prm
    
    if [ $? -eq 0 ]; then
        echo "✅ Workspace compilado com sucesso"
        source install/setup.bash
    else
        echo "❌ Erro na compilação do workspace"
    fi
fi

# Mostrar informações úteis
echo ""
echo "🚀 Container PRM iniciado com sucesso!"
echo "📍 Você está em: $(pwd)"
echo "🔧 Distribuição ROS2: $ROS_DISTRO"
echo "🌍 Gazebo versão: $GZ_VERSION"
echo ""
echo "💡 Comandos úteis:"
echo "   colcon build --symlink-install --packages-select prm  # Compilar o projeto"
echo "   ros2 launch prm inicia_simulacao.launch.py            # Iniciar simulação"
echo "   ros2 launch prm teste_urdf.launch.py                  # Testar URDF"
echo "   ros2 run prm tartaruga                                # Executar controle da tartaruga"
echo ""

# Executar comando passado como argumento ou bash interativo
exec "$@"
