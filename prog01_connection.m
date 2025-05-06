clear, clc, close all

disp('Programa iniciado');
sim = remApi('remoteApi'); % usando o arquivo prototipo (remoteApiProto.m)

sim.simxFinish(-1); % Por precaucao, fecha todas as conexoes abertas
clientID = sim.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connectado a API remota do servidor');
        
    % Agora tentando recuperar dados de forma bloqueante (ou seja, modo de operacao uma chamada de servico):
    [res,objs] = sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking);

    if (res == sim.simx_return_ok)
        fprintf('Numero de objetos na cena: %d\n',length(objs));
    else
        fprintf('Chamada de funcao da API Remota retornou com codigo de erro: %d\n',res);
    end
end
    
fprintf('Tempo de Ping: %d\n', sim.simxGetPingTime(clientID))

