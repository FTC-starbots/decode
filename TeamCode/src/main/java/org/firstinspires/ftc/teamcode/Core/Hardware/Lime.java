package org.firstinspires.ftc.teamcode.Core.Hardware;

public class Lime {
    int pipeline;
    public void start() {
        // Inicia a câmera
    }

    public void stop() {
        // Para a câmera

    }

    public void restart(){
        stop();
        start();
    }

    public void setPipeline(int pipe){
        pipeline = pipe;
    }

    public void update(){

        // Atualiza a imagem da câmera
    }

}
