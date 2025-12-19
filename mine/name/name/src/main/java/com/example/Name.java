package com.example;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.bukkit.Material;
import org.bukkit.Particle;
import org.bukkit.Sound;
import org.bukkit.block.Block;
import org.bukkit.block.BlockState;
import org.bukkit.command.Command;
import org.bukkit.command.CommandSender;
import org.bukkit.entity.Player;
import org.bukkit.event.EventHandler;
import org.bukkit.event.Listener;
import org.bukkit.event.world.PortalCreateEvent;
import org.bukkit.inventory.ItemStack;
import org.bukkit.inventory.meta.ItemMeta;
import org.bukkit.persistence.PersistentDataType;
import org.bukkit.plugin.java.JavaPlugin;
import org.bukkit.scheduler.BukkitRunnable;

import net.md_5.bungee.api.ChatColor;
public class Name extends JavaPlugin implements Listener
{
    @Override
    public void onEnable() {
        getServer().getPluginManager().registerEvents(this, this);
        getLogger().info("Плагин успешно включен!");
    }

    @Override
    public void onDisable() {
        getLogger().info("Плагин выключается...");
    }

    private ItemStack createLoreNote(String text) {
        ItemStack note = new ItemStack(Material.PAPER);
        ItemMeta meta = note.getItemMeta();
        meta.setDisplayName(ChatColor.GOLD + "Таинственная записка");
        meta.setLore(Arrays.asList(text));
        note.setItemMeta(meta);
        return note;
    }

    @Override
    public boolean onCommand(CommandSender sender, Command cmd, String label, String[] args) {
        if (cmd.getName().equalsIgnoreCase("lorenote")) {
            if (!(sender instanceof Player)) {
                sender.sendMessage("Эту команду может использовать только игрок.");
                return true;
            }
            
            if (args.length == 0) {
                sender.sendMessage(ChatColor.RED + "Использование: /lorenote <текст>");
                return true;
            }

            Player player = (Player) sender;
            String message = String.join(" ", args);
            ItemStack item = createLoreNote(message);
            player.getInventory().addItem(item);
            player.sendMessage(ChatColor.GREEN + "Вы получили записку!");
            
            return true;
        }
        return false;
    }

    @EventHandler
    public void onPortalCreate(PortalCreateEvent event) {
        if (event.getReason() != PortalCreateEvent.CreateReason.FIRE) return;
        List<BlockState> blocks = new ArrayList<>(event.getBlocks());
        new BukkitRunnable() {
            @Override
            public void run() {
                for (BlockState state : blocks) {
                    if (state.getType() == Material.OBSIDIAN) {
                        Block realBlock = state.getBlock();
                        realBlock.setType(Material.CRYING_OBSIDIAN); 
                        realBlock.getWorld().playSound(realBlock.getLocation(), Sound.BLOCK_GLASS_BREAK, 1.0f, 0.5f);
                        realBlock.getWorld().spawnParticle(Particle.LAVA, realBlock.getLocation(), 30);
                        break; 
                    }
                }
            }
        }.runTaskLater(this, 1L);
    }
}
